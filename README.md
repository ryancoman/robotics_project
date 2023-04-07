# robotics\_project
Repo for CSC/ECE 591 Robotics software project

## ROS Nodes
There are two ROS nodes in the catkin_ws directory: `process_data_node` and `imu_publisher_node`. The `imu_publisher_node` reads the data from the IMU and publishes it on a `imu_data` topic, where it is then processed by the `process_data_node`.

The steps to compile and launch the nodes are as follows:

1. Install ROS Melodic. Instructions can be found here: http://wiki.ros.org/melodic/Installation/Ubuntu
2. Install the packages needed for tf2_geometry_msgs: `sudo apt install ros-melodic-tf2-geometry-msgs`
3. You may need to install the Boost C++ libraries, but it may work without that installed. I used `sudo apt install libboost-all-dev` to do this.
4. Compile the code. Change to the `catkin_ws` directory, then run `catkin_make`.
5. Run `roscore`.
6. In a separate terminal, source the setup file: from the `catkin_ws` directory, run `source devel/setup.bash`.
7. Launch the nodes with `roslaunch robotic_puppet robotic_puppet.launch`.