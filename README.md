## Calixto-ros-bot

This is the repo of calixto's amr proto. 
version:
ROS HUMBLE, Gazebo 11.10, 

*TO SLAM (Mapping )*

1. ros2 launch calixto-ros-bot launch_sim.launch.py world:=src/calixto-ros-bot/gazebo/maze.world

2. rviz2 

3. ros2 launch slam_toolbox online_async_launch.py slam_params_file:=src/calixto-ros-bot/config/mapper_params_online_async.yaml use_sim_time:=true

[ Add new map, laser_scan , assign topics ]

4. ros2 run teleop_twist_keyboard teleop_twist_keyboard

drive around and create map

5.  Save map using rviz pluggin, serial and pgm

6. change the mapper_param file #mapping to #localisation and map path below it, start at dock true

*TO EXECUTE AUTONOMOUS NAVIGATION PROCESS*

1. To run the Gazebo simulation launch file :

ros2 launch calixto-ros-bot launch_sim.launch.py world:=src/calixto-ros-bot/gazebo/maze.world

2. To run the Rviz2 for Nav2 :
rviz2

3. To launch the slam_toolbox with existing map :

ros2 launch slam_toolbox online_async_launch.py slam_params_file:=src/calixto-ros-bot/config/mapper_params_online_async.yaml use_sim_time:=true

(this method will map and can be used for the navigation purpose also ; map+nav)

4. To launch the Nav2 navigation stack :

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

5. To run the teleop keyboard for navigation (if req.) :

ros2 run teleop_twist_keyboard teleop_twist_keyboard


***** NAV2 with AMCL*****
1. ros2 run twist_mux twist_mux --ros-args --params-file ./src/calixto-ros-bot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped  

// required in the possible future for autonomous + remote driving , mux the input from var sources

2. ros2 launch calixto-ros-bot launch_sim.launch.py world:=src/calixto-ros-bot/gazebo/maze.world

3. ros2 launch nav2_bringup bringup_launch.py     map:=/path/to/map.yaml     use_sim_time:=true     params_file:=/home/anandhu-sudha/ws_ros2/src/calixto-ros-bot/config/nav2_params.yaml

4. rviz2

// load with saved config.

5. ros2 launch nav2_bringup localization_launch.py     map:=/home/anandhu-sudha/ws_ros2/src/calixto-ros-bot/maps/my_map.yaml     use_sim_time:=true     map_subscribe_transient_local:=true

set the initial pose, set the cost map and all, set the target location.
