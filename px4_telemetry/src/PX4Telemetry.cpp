#include "PX4Telemetry.hpp" // Assuming your header file

#define MIN_VOLTAGE 19.2
#define MAX_VOLTAGE 25.2

using std::placeholders::_1;
using namespace std::chrono_literals;

PX4Telemetry::PX4Telemetry() : Node("px4_telemetry_node"),
                               alt_init_(false), lpos_init_(false), gpos_init_(false),
                               current_flight_state_(FlightState::S_INITIALIZING), // Initial FSM state
                               pending_land_after_mode_switch_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing PX4 Telemetry Node");

    px4_id_ = std::string(this->get_namespace()).substr(1);
    std::string utm_band_str;

    this->declare_parameter("origin_x", 0.0);
    this->declare_parameter("origin_y", 0.0);
    this->declare_parameter("origin_r", 0.0);
    this->declare_parameter("utm_zone", 0);
    this->declare_parameter("utm_band", "R");
    if (
        this->get_parameter("origin_x", origin_x_) &&
        this->get_parameter("origin_y", origin_y_) &&
        this->get_parameter("origin_r", origin_r_) &&
        this->get_parameter("utm_zone", utm_zone_) &&
        this->get_parameter("utm_band", utm_band_str)
    ) {
        utm_band_ = utm_band_str[0];
        RCLCPP_INFO(this->get_logger(), "Park origin set to (%.4f, %.4f), %.4f rad, Zone %d, Band %c", origin_x_, origin_y_, origin_r_, utm_zone_, utm_band_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Park geodesy parameters not provided.");
        rclcpp::shutdown();
        return;
    }

    // Initialize button indices from parameters
    this->declare_parameter("arm_button", -1);
    this->declare_parameter("disarm_button", -1);
    this->declare_parameter("takeoff_button", -1);
    this->declare_parameter("land_button", -1);
    this->declare_parameter("offboard_button", -1); // Assuming this was 'control_button' or similar

    this->get_parameter("arm_button", joy_button_indices_.arm);
    this->get_parameter("disarm_button", joy_button_indices_.disarm);
    this->get_parameter("takeoff_button", joy_button_indices_.takeoff);
    this->get_parameter("land_button", joy_button_indices_.land);
    this->get_parameter("offboard_button", joy_button_indices_.offboard);

    RCLCPP_INFO(this->get_logger(), "Loaded joy button parameters: Arm: %d, Disarm: %d, Takeoff: %d, Land: %d, Offboard: %d",
                joy_button_indices_.arm, joy_button_indices_.disarm, joy_button_indices_.takeoff, joy_button_indices_.land, joy_button_indices_.offboard);

    // Initialize previous button states
    prev_button_states_ = {0, 0, 0, 0, 0};


    this->declare_parameter("sim_mode", false);
    this->get_parameter("sim_mode", sim_mode_);
    if (sim_mode_) {
        RCLCPP_WARN(this->get_logger(), "Simulation mode enabled.");
    }

    q_utm_to_apark_.setRPY(0, 0, origin_r_);
    q_apark_to_utm_.setRPY(0, 0, -origin_r_);
    egm96_5_ = std::make_shared<GeographicLib::Geoid>("egm96-5", "", true, true);

    apark_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    apark_tf_.header.frame_id = "autonomy_park";
    apark_tf_.child_frame_id = px4_id_;
    apark_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("autonomy_park/pose", 1);
    apark_pose_.header.frame_id = "autonomy_park";
    gp_origin_publisher_ = this->create_publisher<geographic_msgs::msg::GeoPointStamped>("global_position/set_gp_origin", 1);

    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("set_mode");
    arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("cmd/arming");
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("cmd/takeoff");
    land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("cmd/land");

    // Service availability checks
    if (!set_mode_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(this->get_logger(), "SetMode service not available. Exiting.");
        if (rclcpp::ok()) rclcpp::shutdown(); return;
    }
    if (!arm_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(this->get_logger(), "Arming service not available. Exiting.");
        if (rclcpp::ok()) rclcpp::shutdown(); return;
    }
    // Note: land_client uses the same CommandTOL service as takeoff_client, so one check for takeoff_client_ is enough for CommandTOL.
    if (!takeoff_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(this->get_logger(), "CommandTOL service (for takeoff/land) not available. Exiting.");
        if (rclcpp::ok()) rclcpp::shutdown(); return;
    }


    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&PX4Telemetry::joy_callback, this, _1));

    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);
    sub_qos.best_effort();
    sub_qos.durability_volatile();

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>("state", sub_qos, std::bind(&PX4Telemetry::state_callback, this, _1));
    ext_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>("extended_state", sub_qos, std::bind(&PX4Telemetry::ext_state_callback, this, _1));
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>("battery", sub_qos, std::bind(&PX4Telemetry::battery_callback, this, _1));
    altitude_sub_ = this->create_subscription<mavros_msgs::msg::Altitude>("altitude", sub_qos, std::bind(&PX4Telemetry::altitude_callback, this, _1));
    global_lpos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("global_position/local", sub_qos, std::bind(&PX4Telemetry::global_lpos_callback, this, _1));
    global_gpos_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("global_position/global", sub_qos, std::bind(&PX4Telemetry::global_gpos_callback, this, _1));

    loiter_str_ = std::string("AUTO.LOITER");
    offboard_str_ = std::string("OFFBOARD");

    initialize_fsm(); // Initialize FSM state
    RCLCPP_INFO(this->get_logger(), "PX4 Telemetry Initialized. Current FSM State: %s", flight_state_to_string(current_flight_state_).c_str());
}

void PX4Telemetry::initialize_fsm() {
    current_flight_state_ = FlightState::S_INITIALIZING;
    pending_land_after_mode_switch_ = false;
    // Log initial state
    RCLCPP_INFO(this->get_logger(), "FSM initialized. State: %s", flight_state_to_string(current_flight_state_).c_str());
}

std::string PX4Telemetry::flight_state_to_string(FlightState state) {
    switch (state) {
        case FlightState::S_INITIALIZING: return "INITIALIZING";
        case FlightState::S_GROUND_DISARMED: return "GROUND_DISARMED";
        case FlightState::S_GROUND_ARMED: return "GROUND_ARMED";
        case FlightState::S_TAKING_OFF: return "TAKING_OFF";
        case FlightState::S_LOITERING: return "LOITERING";
        case FlightState::S_OFFBOARD_CONTROL: return "OFFBOARD_CONTROL";
        case FlightState::S_LANDING: return "LANDING";
        default: return "UNKNOWN_STATE";
    }
}

void PX4Telemetry::transition_to_state(FlightState new_state) {
    if (current_flight_state_ != new_state) {
        RCLCPP_INFO(this->get_logger(), "FSM Transition: %s -> %s",
                    flight_state_to_string(current_flight_state_).c_str(),
                    flight_state_to_string(new_state).c_str());
        current_flight_state_ = new_state;

        // Handle entry actions for new states if necessary
        if (new_state == FlightState::S_LOITERING) {
            if (pending_land_after_mode_switch_) {
                RCLCPP_INFO(this->get_logger(), "Switched to LOITER, now initiating pending land command.");
                pending_land_after_mode_switch_ = false;
                send_land_command();
            }
        }
    }
}

void PX4Telemetry::check_and_transition_if_telemetry_ready() {
    if (current_flight_state_ == FlightState::S_INITIALIZING) {
        if (alt_init_ && lpos_init_ && gpos_init_) {
            RCLCPP_INFO(this->get_logger(), "All telemetry initialised.");
            // Initial transition based on current PX4 state (assuming it starts disarmed on ground)
            if (!current_px4_state_.armed && landed_state_ == LandedState::on_ground) {
                 transition_to_state(FlightState::S_GROUND_DISARMED);
            } else if (current_px4_state_.armed && landed_state_ == LandedState::on_ground) {
                 transition_to_state(FlightState::S_GROUND_ARMED);
            } else {
                // If PX4 is in an unexpected state, log it. May need manual recovery or specific handling.
                RCLCPP_WARN(this->get_logger(), "Telemetry ready, but PX4 state is unexpected (Armed: %d, LandedState: %d). Remaining in INITIALIZING or a suitable error state might be needed.", current_px4_state_.armed, static_cast<int>(landed_state_));
                // For now, we'll default to ground disarmed if telemetry is ready but PX4 state isn't immediately clear or ideal
                // This part might need refinement based on how quickly PX4 state messages arrive relative to telemetry.
                // A robust way is to wait for first state_callback and ext_state_callback before this transition.
                // Or, state_callback and ext_state_callback can also call this check.
                // Let's assume state_callback and ext_state_callback will finalize this transition.
            }
        }
    }
}


void PX4Telemetry::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    if (current_flight_state_ == FlightState::S_INITIALIZING) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Joystick inputs ignored: Telemetry not yet initialized or FSM in INITIALIZING state.");
        return;
    }

    UserCommand command = UserCommand::CMD_NONE;

    // ARM Command
    if (joy_button_indices_.arm >= 0 && joy_button_indices_.arm < (int)joy_msg->buttons.size()) {
        if (joy_msg->buttons[joy_button_indices_.arm] == 1 && prev_button_states_.arm == 0) {
            command = UserCommand::CMD_ARM;
        }
        prev_button_states_.arm = joy_msg->buttons[joy_button_indices_.arm];
    }

    // DISARM Command
    if (joy_button_indices_.disarm >= 0 && joy_button_indices_.disarm < (int)joy_msg->buttons.size()) {
        if (joy_msg->buttons[joy_button_indices_.disarm] == 1 && prev_button_states_.disarm == 0) {
            command = UserCommand::CMD_DISARM;
        }
        prev_button_states_.disarm = joy_msg->buttons[joy_button_indices_.disarm];
    }

    // TAKEOFF Command
    if (joy_button_indices_.takeoff >= 0 && joy_button_indices_.takeoff < (int)joy_msg->buttons.size()) {
        if (joy_msg->buttons[joy_button_indices_.takeoff] == 1 && prev_button_states_.takeoff == 0) {
            command = UserCommand::CMD_TAKEOFF;
        }
        prev_button_states_.takeoff = joy_msg->buttons[joy_button_indices_.takeoff];
    }

    // LAND Command
    if (joy_button_indices_.land >= 0 && joy_button_indices_.land < (int)joy_msg->buttons.size()) {
        if (joy_msg->buttons[joy_button_indices_.land] == 1 && prev_button_states_.land == 0) {
            command = UserCommand::CMD_LAND;
        }
        prev_button_states_.land = joy_msg->buttons[joy_button_indices_.land];
    }

    // TOGGLE OFFBOARD Command
    if (joy_button_indices_.offboard >= 0 && joy_button_indices_.offboard < (int)joy_msg->buttons.size()) {
        if (joy_msg->buttons[joy_button_indices_.offboard] == 1 && prev_button_states_.offboard == 0) {
            command = UserCommand::CMD_TOGGLE_OFFBOARD;
        }
        prev_button_states_.offboard = joy_msg->buttons[joy_button_indices_.offboard];
    }

    if (command != UserCommand::CMD_NONE) {
        process_user_command(command);
    }
}

void PX4Telemetry::process_user_command(UserCommand command) {
    RCLCPP_DEBUG(this->get_logger(), "Processing user command %d in state %s", static_cast<int>(command), flight_state_to_string(current_flight_state_).c_str());

    switch (current_flight_state_) {
        case FlightState::S_GROUND_DISARMED:
            if (command == UserCommand::CMD_ARM) {
                send_arm_disarm_command(true);
            } else {
                RCLCPP_WARN(this->get_logger(), "Command %d ignored in state GROUND_DISARMED.", static_cast<int>(command));
            }
            break;

        case FlightState::S_GROUND_ARMED:
            if (command == UserCommand::CMD_DISARM) {
                send_arm_disarm_command(false);
            } else if (command == UserCommand::CMD_TAKEOFF) {
                send_takeoff_command();
            } else {
                RCLCPP_WARN(this->get_logger(), "Command %d ignored in state GROUND_ARMED.", static_cast<int>(command));
            }
            break;

        case FlightState::S_LOITERING:
            if (command == UserCommand::CMD_LAND) {
                send_land_command();
            } else if (command == UserCommand::CMD_TOGGLE_OFFBOARD) {
                request_flight_mode(offboard_str_);
            } else {
                RCLCPP_WARN(this->get_logger(), "Command %d ignored in state LOITERING.", static_cast<int>(command));
            }
            break;

        case FlightState::S_OFFBOARD_CONTROL:
            if (command == UserCommand::CMD_LAND) {
                RCLCPP_INFO(this->get_logger(), "Land requested from OFFBOARD. Switching to LOITER first.");
                pending_land_after_mode_switch_ = true;
                request_flight_mode(loiter_str_); // Transition to S_LOITERING, then land will be triggered.
            } else if (command == UserCommand::CMD_TOGGLE_OFFBOARD) {
                request_flight_mode(loiter_str_);
            } else {
                RCLCPP_WARN(this->get_logger(), "Command %d ignored in state OFFBOARD_CONTROL.", static_cast<int>(command));
            }
            break;

        case FlightState::S_TAKING_OFF:
        case FlightState::S_LANDING:
            RCLCPP_WARN(this->get_logger(), "Command %d ignored: drone is busy %s.", static_cast<int>(command), flight_state_to_string(current_flight_state_).c_str());
            break;
        
        case FlightState::S_INITIALIZING:
             RCLCPP_WARN(this->get_logger(), "Command %d ignored: FSM still in INITIALIZING state.", static_cast<int>(command));
            break;

        default:
            RCLCPP_ERROR(this->get_logger(), "Unhandled command %d in unknown state %s.", static_cast<int>(command), flight_state_to_string(current_flight_state_).c_str());
            break;
    }
}

void PX4Telemetry::state_callback(const mavros_msgs::msg::State::SharedPtr state_msg) {
    bool prev_armed = current_px4_state_.armed;
    std::string prev_mode = current_px4_state_.mode;
    current_px4_state_ = *state_msg; // Store the new state

    if (current_flight_state_ == FlightState::S_INITIALIZING && (alt_init_ && lpos_init_ && gpos_init_)) {
        // If telemetry is ready, try to move to an operational ground state based on first PX4 state
        if (!current_px4_state_.armed && landed_state_ == LandedState::on_ground) {
             transition_to_state(FlightState::S_GROUND_DISARMED);
        } else if (current_px4_state_.armed && landed_state_ == LandedState::on_ground) {
             transition_to_state(FlightState::S_GROUND_ARMED);
        }
    }


    // Armed state change
    if (state_msg->armed && !prev_armed) {
        RCLCPP_WARN(this->get_logger(), "PX4 Reported: ARMED");
        if (current_flight_state_ == FlightState::S_GROUND_DISARMED && landed_state_ == LandedState::on_ground) {
            transition_to_state(FlightState::S_GROUND_ARMED);
        }
    } else if (!state_msg->armed && prev_armed) {
        RCLCPP_WARN(this->get_logger(), "PX4 Reported: DISARMED");
        if (current_flight_state_ == FlightState::S_GROUND_ARMED && landed_state_ == LandedState::on_ground) {
            transition_to_state(FlightState::S_GROUND_DISARMED);
        } else if (landed_state_ != LandedState::on_ground) {
            RCLCPP_ERROR(this->get_logger(), "Disarmed while in air! This is critical. Transitioning to GROUND_DISARMED.");
            // This is an emergency situation, force to a ground state.
            // Actual drone behavior depends on PX4 failsafes.
            transition_to_state(FlightState::S_GROUND_DISARMED);
        }
    }

    // Mode change
    if (state_msg->mode != prev_mode) {
        RCLCPP_WARN(this->get_logger(), "PX4 Reported: Mode changed from '%s' to '%s'", prev_mode.c_str(), state_msg->mode.c_str());
        if (state_msg->mode == loiter_str_) {
            if (current_flight_state_ == FlightState::S_OFFBOARD_CONTROL ||
                (current_flight_state_ == FlightState::S_TAKING_OFF && landed_state_ == LandedState::in_air) || /* After takeoff, PX4 might go to LOITER */
                 current_flight_state_ == FlightState::S_INITIALIZING /* If initializing directly into loiter */ ) {
                transition_to_state(FlightState::S_LOITERING);
            }
        } else if (state_msg->mode == offboard_str_) {
            if (current_flight_state_ == FlightState::S_LOITERING && landed_state_ == LandedState::in_air) {
                transition_to_state(FlightState::S_OFFBOARD_CONTROL);
            } else if (landed_state_ == LandedState::on_ground) {
                 RCLCPP_ERROR(this->get_logger(), "PX4 switched to OFFBOARD while on ground. This is usually disallowed. Switching back to LOITER attempt.");
                 request_flight_mode(loiter_str_); // Attempt to revert to a safer mode
                 // FSM state might need to be S_GROUND_ARMED or S_GROUND_DISARMED here, not S_OFFBOARD_CONTROL
            }
        }
    }
}

void PX4Telemetry::ext_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr ext_state_msg) {
    LandedState prev_landed_state = landed_state_;
    landed_state_ = (LandedState)ext_state_msg->landed_state;

    if (current_flight_state_ == FlightState::S_INITIALIZING && (alt_init_ && lpos_init_ && gpos_init_)) {
         // If telemetry is ready, try to move to an operational ground state
        if (!current_px4_state_.armed && landed_state_ == LandedState::on_ground) {
             transition_to_state(FlightState::S_GROUND_DISARMED);
        } else if (current_px4_state_.armed && landed_state_ == LandedState::on_ground) {
             transition_to_state(FlightState::S_GROUND_ARMED);
        }
    }

    if (landed_state_ != prev_landed_state) {
        RCLCPP_INFO(this->get_logger(), "PX4 Reported: Landed state changed to %d", static_cast<int>(landed_state_));
        switch (landed_state_) {
            case LandedState::on_ground:
                RCLCPP_WARN(this->get_logger(), "Landed state: ON_GROUND.");
                if (current_flight_state_ == FlightState::S_LANDING || current_flight_state_ == FlightState::S_TAKING_OFF /*aborted takeoff*/) {
                    if (current_px4_state_.armed) {
                        transition_to_state(FlightState::S_GROUND_ARMED);
                    } else {
                        transition_to_state(FlightState::S_GROUND_DISARMED);
                    }
                } else if (current_flight_state_ == FlightState::S_INITIALIZING && !current_px4_state_.armed){
                     transition_to_state(FlightState::S_GROUND_DISARMED);
                } else if (current_flight_state_ == FlightState::S_INITIALIZING && current_px4_state_.armed){
                     transition_to_state(FlightState::S_GROUND_ARMED);
                }
                break;
            case LandedState::in_air:
                RCLCPP_WARN(this->get_logger(), "Landed state: IN_AIR.");
                if (current_flight_state_ == FlightState::S_TAKING_OFF) {
                    // Assuming PX4 switches to LOITER after takeoff command completes and drone is in air.
                    // The state_callback for mode change to LOITER will handle transition to S_LOITERING.
                    // If PX4 stays in a "takeoff" mode string, this logic might need adjustment.
                    RCLCPP_INFO(this->get_logger(), "Takeoff procedure seems complete, drone in air. Awaiting mode confirmation for LOITER/HOLD.");
                    if(current_px4_state_.mode == loiter_str_) { // If already in loiter
                        transition_to_state(FlightState::S_LOITERING);
                    }
                    // Otherwise, state_callback will handle it when mode changes
                }
                break;
            case LandedState::takeoff:
                RCLCPP_WARN(this->get_logger(), "Landed state: TAKEOFF.");
                if (current_flight_state_ == FlightState::S_GROUND_ARMED || current_flight_state_ == FlightState::S_LOITERING /*e.g. re-takeoff cmd*/) {
                    transition_to_state(FlightState::S_TAKING_OFF);
                }
                break;
            case LandedState::landing:
                RCLCPP_WARN(this->get_logger(), "Landed state: LANDING.");
                if (current_flight_state_ == FlightState::S_LOITERING || current_flight_state_ == FlightState::S_OFFBOARD_CONTROL) {
                    // OFFBOARD should have switched to LOITER before land command was effective
                    transition_to_state(FlightState::S_LANDING);
                }
                break;
            case LandedState::undefined:
                RCLCPP_ERROR(this->get_logger(), "Landed state: UNDEFINED!");
                break;
        }
    }
}


void PX4Telemetry::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    battery_voltage_ = msg->voltage;
    // RCLCPP_DEBUG(this->get_logger(), "Battery = %.2f%%", (msg->voltage-MIN_VOLTAGE)/(MAX_VOLTAGE-MIN_VOLTAGE)*100.0);
}

void PX4Telemetry::altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg) {
    if (sim_mode_) {
        apark_pose_.pose.position.z = msg->monotonic;
    } else {
        apark_pose_.pose.position.z = msg->local;
    }
    altitude_amsl_ = msg->amsl;
    if (!alt_init_) {
        alt_init_ = true;
        check_and_transition_if_telemetry_ready();
    }
}

void PX4Telemetry::global_lpos_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q_utm, q_apark;
    tf2::fromMsg(msg->pose.pose.orientation, q_utm);
    q_apark = q_utm_to_apark_ * q_utm;
    apark_pose_.pose.orientation = tf2::toMsg(q_apark);
    if (!lpos_init_) {
        lpos_init_ = true;
        check_and_transition_if_telemetry_ready();
    }
}

void PX4Telemetry::global_gpos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    rclcpp::Time now = this->get_clock()->now();
    auto geo_msg = geographic_msgs::msg::GeoPoint();
    geo_msg.latitude = msg->latitude;
    geo_msg.longitude = msg->longitude;
    geo_msg.altitude = msg->altitude;

    geodesy::UTMPoint utm_pos;
    geodesy::fromMsg(geo_msg, utm_pos);

    double dx = utm_pos.easting - origin_x_;
    double dy = utm_pos.northing - origin_y_;
    apark_pose_.pose.position.x = cos(origin_r_) * dx - sin(origin_r_) * dy;
    apark_pose_.pose.position.y = sin(origin_r_) * dx + cos(origin_r_) * dy;
    apark_pose_.header.stamp = now;
    this->apark_pose_publisher_->publish(apark_pose_);

    apark_tf_.header.stamp = now;
    apark_tf_.transform.translation.x = apark_pose_.pose.position.x;
    apark_tf_.transform.translation.y = apark_pose_.pose.position.y;
    apark_tf_.transform.translation.z = apark_pose_.pose.position.z;
    apark_tf_.transform.rotation = apark_pose_.pose.orientation;
    apark_tf_broadcaster_->sendTransform(apark_tf_);

    if (!gpos_init_) {
        gpos_init_ = true;
        check_and_transition_if_telemetry_ready();
    }
}

// Original get_button can be removed if joy_button_indices_ is used directly in joy_callback
// For now, it's unused by the new joy_callback logic.
// If you still need it for other purposes, ensure Button struct and its usage are consistent.
// int PX4Telemetry::get_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Button &button) { ... }


void PX4Telemetry::send_arm_disarm_command(bool arm) {
    // FSM should ensure this is called in a valid state.
    // This function now mainly sends the command.
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_request->value = arm;
    if (arm) {
        RCLCPP_INFO(this->get_logger(), "Sending ARM request to PX4.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Sending DISARM request to PX4.");
    }
    arm_client_->async_send_request(arm_request, std::bind(&PX4Telemetry::arm_response_callback, this, _1));
}

void PX4Telemetry::send_takeoff_command() {
    // FSM ensures this is called in S_GROUND_ARMED.
    auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    
    // For simplicity, takeoff to a relative altitude above current.
    // PX4's takeoff service usually uses AMSL altitude.
    // Ensure altitude_amsl_ is up-to-date.
    // Taking off to 2 meters above current AMSL.
    takeoff_request->altitude = altitude_amsl_ + 2.0; 
    // Yaw, lat, long can be current or NaN to use current PX4 values if supported.
    // For MAVROS takeoff, typically you provide current lat/long.
    geographic_msgs::msg::GeoPose global_pose = apark_to_global(apark_pose_.pose); // Use current pose
    takeoff_request->latitude = global_pose.position.latitude;
    takeoff_request->longitude = global_pose.position.longitude;
    takeoff_request->yaw = quat_to_yaw(global_pose.orientation); // Current yaw

    RCLCPP_INFO(this->get_logger(), "Sending TAKEOFF request to PX4 (Lat: %.6f, Lon: %.6f, Alt_AMSL: %.2f, Yaw: %.2f).",
                takeoff_request->latitude, takeoff_request->longitude, takeoff_request->altitude, takeoff_request->yaw);
    takeoff_client_->async_send_request(takeoff_request, std::bind(&PX4Telemetry::tol_response_callback, this, _1, "Takeoff"));
}

void PX4Telemetry::send_land_command() {
    // FSM ensures this is called from S_LOITERING or S_OFFBOARD_CONTROL (after switch to loiter).
    auto land_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    // Land at current location. MAVROS land might not need altitude, or uses 0 for current ground level.
    // For safety, providing current lat/lon is good. Altitude is often ignored or set to current ground.
    geographic_msgs::msg::GeoPose global_pose = apark_to_global(apark_pose_.pose);
    land_request->latitude = global_pose.position.latitude;
    land_request->longitude = global_pose.position.longitude;
    land_request->yaw = quat_to_yaw(global_pose.orientation); // Current yaw
    // land_request->altitude = 0; // Or NaN, depending on PX4/MAVROS version specifics for land

    RCLCPP_INFO(this->get_logger(), "Sending LAND request to PX4 (Lat: %.6f, Lon: %.6f, Yaw: %.2f).",
                land_request->latitude, land_request->longitude, land_request->yaw);
    land_client_->async_send_request(land_request, std::bind(&PX4Telemetry::tol_response_callback, this, _1, "Land"));
}

void PX4Telemetry::request_flight_mode(const std::string& mode) {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    RCLCPP_INFO(this->get_logger(), "Requesting PX4 mode: %s", mode.c_str());

    if (mode == offboard_str_) {
        set_mode_client_->async_send_request(request, std::bind(&PX4Telemetry::offboard_mode_response_callback, this, _1));
    } else if (mode == loiter_str_) {
        set_mode_client_->async_send_request(request, std::bind(&PX4Telemetry::loiter_mode_response_callback, this, _1));
    } else {
        RCLCPP_ERROR(this->get_logger(), "Attempted to request unknown mode: %s", mode.c_str());
    }
}

void PX4Telemetry::loiter_mode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
    auto response = future.get();
    if (response->mode_sent) {
        RCLCPP_INFO(this->get_logger(), "Loiter mode request SUCCEEDED (sent to PX4).");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Loiter mode request FAILED (not sent to PX4).");
        // If a land was pending, this is problematic.
        if (pending_land_after_mode_switch_) {
            RCLCPP_ERROR(this->get_logger(), "Mode switch to LOITER failed, cannot proceed with pending LAND command.");
            pending_land_after_mode_switch_ = false; // Clear flag as we can't proceed
        }
    }
}

void PX4Telemetry::offboard_mode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
    auto response = future.get();
    if (response->mode_sent) {
        RCLCPP_INFO(this->get_logger(), "Offboard mode request SUCCEEDED (sent to PX4).");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Offboard mode request FAILED (not sent to PX4).");
    }
}

void PX4Telemetry::arm_response_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
    auto response = future.get();
    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Arm/Disarm request to PX4 SUCCEEDED. Result=%d", response->result);
        // FSM transition will occur based on state_callback from PX4, not directly here.
    } else {
        RCLCPP_ERROR(this->get_logger(), "Arm/Disarm request to PX4 FAILED!");
    }
}

// Combined TOL response callback
void PX4Telemetry::tol_response_callback(rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future, const std::string& tol_type) {
    auto response = future.get();
    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "%s request to PX4 SUCCEEDED. Result=%d", tol_type.c_str(), response->result);
        // FSM transition will occur based on ext_state_callback from PX4
    } else {
        RCLCPP_ERROR(this->get_logger(), "%s request to PX4 FAILED!", tol_type.c_str());
    }
}


double PX4Telemetry::quat_to_yaw(geometry_msgs::msg::Quaternion quat) {
    double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

geographic_msgs::msg::GeoPose PX4Telemetry::apark_to_global(const geometry_msgs::msg::Pose &apark_pose_msg) {
    double sp_x = apark_pose_msg.position.x;
    double sp_y = apark_pose_msg.position.y;

    double dx = cos(origin_r_) * sp_x + sin(origin_r_) * sp_y;
    double dy = -sin(origin_r_) * sp_x + cos(origin_r_) * sp_y;

    geodesy::UTMPoint utm_pos;
    utm_pos.zone = utm_zone_;
    utm_pos.band = utm_band_;
    utm_pos.easting = dx + origin_x_;
    utm_pos.northing = dy + origin_y_;

    geographic_msgs::msg::GeoPoint global_pos_msg = geodesy::toMsg(utm_pos);
    global_pos_msg.altitude = altitude_amsl_; // Use current AMSL for commands

    tf2::Quaternion q_apark_tf, q_utm_tf;
    tf2::fromMsg(apark_pose_msg.orientation, q_apark_tf);
    q_utm_tf = q_apark_to_utm_ * q_apark_tf;

    geographic_msgs::msg::GeoPose global_pose;
    global_pose.position = global_pos_msg;
    global_pose.orientation = tf2::toMsg(q_utm_tf);

    return global_pose;
}