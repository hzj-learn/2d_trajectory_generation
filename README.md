# 2d_trajectory_generation
A ROS repository for receding horizon planning of ground robot

This repository contains the code fo receding horizon trajectory generation and control of ground vehicle. The trajectories are generaed using Pontryagin's 
miniimum principle and the algorithms are tested in simulations and on clearpath jackal. 

This package needs the following packages: 
1. PCL library (version 1.11 or above) 
2. GSL library (uses some functions for trajectory generation) 
3. Eigen3
4. jackal velodyne simulator that can be found at https://github.com/TixiaoShan/jackal_velodyne


Instructions to install all these libraries are straightforward and can be found online. 
The master branch can be used with simulations while the branch titled jackal_navigation has been tested on the real hardware. 

To install this package, navigate to your catkin_ws/src folder and then run: 

git clone https://github.com/indsy123/2d_trajectory_generation

cd ..

catkin build -DCMAKE_BUILD_TYPE=Release

To run the planner for end to end navigation:

roslaunch 2d_trajectory_generation jackal_navigation.launch 

