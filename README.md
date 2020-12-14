#UD_GO_CHASE_IT

A project of udacity nano-degree Robotics Software Engineer. 

A simple robot build in Gazebo with lidar and camera follows a white ball. 

#Tools 
1. ROS
2. GAZEBO
3. C++ 

# Build
1. create catkin_ws
2. clone this repository
3. catkin_make

# launch  and run 
open terminal  
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch my_robot world.launch

Another  terminal  
$ cd catkin_ws
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch


