# robotics_project

Repo for CSC/ECE 591 Robotics software project

## ROS Nodes

There are two ROS nodes in the catkin_ws directory: `process_data_node` and `imu_publisher_node`. The `imu_publisher_node` reads the data from the IMU and publishes it on a `imu_data` topic, where it is then processed by the `process_data_node`.

The steps to compile are as follows:

1. Install ROS Melodic. Instructions can be found here: http://wiki.ros.org/melodic/Installation/Ubuntu
2. Install the packages needed for tf2_geometry_msgs: `sudo apt install ros-melodic-tf2-geometry-msgs`
3. You may need to install the Boost C++ libraries, but it may also work without that installed. I used `sudo apt install libboost-all-dev` to do this.
4. Update the device path in the `imu_publisher_node`. Use the path to the device in the `/dev` directory, and change the value of the `DEVICE` macro.
5. Compile the code. Change to the `catkin_ws` directory, then run `catkin_make`.

The steps to run are as follows:

1. Run `roscore`.
2. In a separate terminal, source the setup file: from the `catkin_ws` directory, run `source devel/setup.bash`.
3. Launch the nodes with `roslaunch robotic_puppet robotic_puppet.launch`.
4. Move the arm to the home position: `rosservice call /j2n6s300_driver/in/home_arm`
5. Launch the Kinova driver: `roslaunch kinova_bringup kinova_robot.launch`
6. Launch the cartesian control node: `rosrun kinova_demo ryan_pose_action_cient.py`
