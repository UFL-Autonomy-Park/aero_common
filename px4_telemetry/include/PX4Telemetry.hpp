#ifndef PX4_TELEMETRY_HPP
#define PX4_TELEMETRY_HPP

#include <rclcpp/rclcpp.hpp>

//TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // Already includes <geometry_msgs/msg/transform_stamped.hpp>

#include <geodesy/utm.h>
#include <GeographicLib/Geoid.hpp>

//Message types
#include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp> // Included by tf2_geometry_msgs
#include <geographic_msgs/msg/geo_point.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/joy.hpp>

//Mavros message and service types
#include <mavros_msgs/msg/altitude.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <string> // For std::string
#include <vector> // If needed for button arrays, etc.

class PX4Telemetry : public rclcpp::Node {
private:
    // FSM States
    enum class FlightState {
        S_INITIALIZING,
        S_GROUND_DISARMED,
        S_GROUND_ARMED,
        S_TAKING_OFF,
        S_LOITERING,
        S_OFFBOARD_CONTROL,
        S_LANDING
    };

    // User Commands from Joystick
    enum class UserCommand {
        CMD_NONE,
        CMD_ARM,
        CMD_DISARM,
        CMD_TAKEOFF,
        CMD_LAND,
        CMD_TOGGLE_OFFBOARD
    };

    // PX4 Landed State (from mavros_msgs::msg::ExtendedState)
    enum LandedState {
        undefined = 0, // mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED
        on_ground = 1, // mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND
        in_air = 2,    // mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR
        takeoff = 3,   // mavros_msgs::msg::ExtendedState::LANDED_STATE_TAKEOFF
        landing = 4    // mavros_msgs::msg::ExtendedState::LANDED_STATE_LANDING
    };

    // ROS Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr altitude_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_lpos_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_gpos_sub_;

    // ROS Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr apark_pose_publisher_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr gp_origin_publisher_;

    // ROS Service Clients
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_; // Kept separate as in original

    // TF and Pose members
    geometry_msgs::msg::PoseStamped apark_pose_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> apark_tf_broadcaster_;
    geometry_msgs::msg::TransformStamped apark_tf_;
    tf2::Quaternion q_utm_to_apark_, q_apark_to_utm_;

    // Geodetic members
    std::string px4_id_;
    double origin_x_, origin_y_, origin_r_;
    uint8_t utm_zone_;
    char utm_band_;
    std::shared_ptr<GeographicLib::Geoid> egm96_5_;

    // Drone State Variables
    mavros_msgs::msg::State current_px4_state_; // Current state from PX4 (mavros_msgs::msg::State)
    LandedState landed_state_;                 // Current landed state from PX4 (mavros_msgs::msg::ExtendedState)
    std::string loiter_str_, offboard_str_;
    double altitude_amsl_;
    double battery_voltage_;
    bool alt_init_, lpos_init_, gpos_init_;    // Telemetry initialization flags
    bool sim_mode_;

    // FSM related members
    FlightState current_flight_state_;
    bool pending_land_after_mode_switch_;

    // Joystick button handling
    struct JoyButtonIndices {
        int arm = -1;
        int disarm = -1;
        int takeoff = -1;
        int land = -1;
        int offboard = -1;
    } joy_button_indices_;

    struct PrevButtonStates { // For edge detection
        int arm = 0;
        int disarm = 0;
        int takeoff = 0;
        int land = 0;
        int offboard = 0;
    } prev_button_states_;

    // Callback functions for subscriptions
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void state_callback(const mavros_msgs::msg::State::SharedPtr state_msg);
    void ext_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr ext_state_msg);
    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg);
    void global_lpos_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void global_gpos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    // FSM handling methods
    void initialize_fsm();
    void transition_to_state(FlightState new_state);
    void process_user_command(UserCommand command);
    void check_and_transition_if_telemetry_ready();
    std::string flight_state_to_string(FlightState state); // For logging

    // Action methods (calling MAVROS services)
    void send_arm_disarm_command(bool arm);
    void send_takeoff_command();
    void send_land_command();
    void request_flight_mode(const std::string& mode);

    // Service response callbacks
    void loiter_mode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);
    void offboard_mode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);
    void arm_response_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future);
    void tol_response_callback(rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future, const std::string& tol_type); // Signature updated

    // Utility functions
    geographic_msgs::msg::GeoPose apark_to_global(const geometry_msgs::msg::Pose &apark_pose_msg); // Parameter name updated for clarity
    double quat_to_yaw(geometry_msgs::msg::Quaternion quat);

public:
    PX4Telemetry();
};

#endif // PX4_TELEMETRY_HPP