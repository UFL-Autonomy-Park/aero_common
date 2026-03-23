<h1 align="center"> Autonomy Sim
</h1>
<div align="center">
   
![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)

A ROS2 metapackage providing the common infrastructure needed to run both single and multi-agent autonomousb UAV flight through PX4 and MAVROS.
It resembles a flight stack, with safety, telemetry logging, teleoperation, and visualization. All packages are included as git submodules and will need to be pulled in to run the full stack.

<img alt="Trajectory Tracking Gif" src="docs/trajectory_tracking.gif"> </img>
</div>

## Packages

| Name | Description |
|---|---|
   | autonomy_park_viz | RViz2 visualization with park geometry. |
   | minimal_startup_air | Contains launch files and startup scripts for sim + physical experiments. |
   | px4_safety_lib | Potential field based safety library for perimeter and obstacle avoidance. |
   | px4_telemetry | Handles telemetry data and frame transformations from ENU to Autonomy Park. |
   | px4_teleop | Teleoperation node. |
   | swarm_interfaces | Library with msgs + srvs used for experiments. |

## Note on Submodules

Submodules will be empty after cloning repo, to update them run:
```bash
git submodule update --init --recursive
```
