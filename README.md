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
