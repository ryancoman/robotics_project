#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
// To get the following header working: sudo apt install ros-melodic-tf2-geometry-msgs
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>

void imuDataCallback(const geometry_msgs::Quaternion::ConstPtr &msg);
void printQuat(tf2::Quaternion *quat);

tf2::Quaternion quat_offset(0.7071068, 0, 0, 0.7071068);
tf2::Quaternion current_rotation(0, 0, 0, 1);
tf2::Quaternion current_calibrated_rotation(0, 0, 0, 1);
tf2::Quaternion current_uncalibrated_rotation(0, 0, 0, 1);
tf2::Quaternion last_calibrated_rotation;
tf2::Quaternion start_quat;
tf2::Quaternion rotation_quaternion;
ros::Publisher pub;

// Time so that too many data points are not sent
ros::WallTime last_sent;

using namespace std;
int main(int argc, char **argv)
{

    // Set the start_quat from euler angles (RPY)
    start_quat.setEuler(180, 0, -30);

    // ROS Setup
    ros::init(argc, argv, "process_data");
    ros::NodeHandle handle;
    pub = handle.advertise<geometry_msgs::Pose>("desired_arm_pose", 10);

    ros::Subscriber sub = handle.subscribe("imu_data", 1000, imuDataCallback);

    // Set time
    last_sent = ros::WallTime::now();

    // According to this (https://answers.ros.org/question/208587/shared-variable-between-callbacks/) page,
    // using ros::spin(); with global variables IS safe because only a single thread handles callbacks.
    // Therefore, in imuDataCallback() it is fine to use and modify globals
    ros::spin();
}

void imuDataCallback(const geometry_msgs::Quaternion::ConstPtr &msg)
{
    // Convert from the message quaternion to a useable type
    tf2::Quaternion new_rotation;
    tf2::fromMsg(*msg, new_rotation);
    current_uncalibrated_rotation = new_rotation;
    new_rotation = quat_offset * new_rotation;
    current_calibrated_rotation = new_rotation;

    // printQuat(&current_calibrated_rotation);

    // We don't want to send too much data, so only send if over 2 seconds have elapsed.
    ros::WallDuration elapsed = ros::WallTime::now() - last_sent;
    if (elapsed.toSec() >= 2)
    {
        geometry_msgs::Pose pose;
        tf2::convert(current_calibrated_rotation, pose.orientation);
        pose.position.x = 0.1;
        pose.position.y = -0.6;
        pose.position.z = 0.4;
        pub.publish(pose);
        last_sent = ros::WallTime::now();
    }
}

/**
 * Convenience function for debugging
 */
void printQuat(tf2::Quaternion *quat)
{
    std::cout << "Quaternion: (" << quat->getX() << ", " << quat->getY() << ", " << quat->getZ() << ", " << quat->getW() << ")" << std::endl;
}