#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <boost/math/quaternion.hpp>

void imuDataCallback(const geometry_msgs::Quaternion::ConstPtr &msg);

using namespace boost::math;

quaternion<double> quat_offset(0, 0, 0, 1);

int main(int argc, char **argv)
{
    // ROS Setup
    ros::init(argc, argv, "process_data");
    ros::NodeHandle handle;

    ros::Subscriber sub = handle.subscribe("imu_data", 1000, imuDataCallback);

    // According to this (https://answers.ros.org/question/208587/shared-variable-between-callbacks/) page,
    // using ros::spin(); with global variables IS safe because only a single thread handles callbacks.
    // Therefore, in imuDataCallback() it is fine to use and modify globals
    ros::spin();
}

void imuDataCallback(const geometry_msgs::Quaternion::ConstPtr &msg)
{
    std::cout << "hello world" << std::endl;
}