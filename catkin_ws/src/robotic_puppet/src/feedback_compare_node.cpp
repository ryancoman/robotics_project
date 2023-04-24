#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
// To get the following header working: sudo apt install ros-melodic-tf2-geometry-msgs
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>


void imuDataCallback(const geometry_msgs::Quaternion::ConstPtr &msg);
void feedbackDataCallback(const geometry_msgs::Quaternion::ConstPtr &msg);
void printQuat(tf2::Quaternion *quat);

tf2::Quaternion quat_offset(0.7071068, 0, 0, 0.7071068);
tf2::Quaternion quat_bais(0, 0, 0, 1);
tf2::Quaternion current_rotation(0, 0, 0, 1);
tf2::Quaternion current_calibrated_rotation_desired(0, 0, 0, 1);
tf2::Quaternion current_uncalibrated_rotation_desired(0, 0, 0, 1);
tf2::Quaternion current_calibrated_rotation_feedback(0, 0, 0, 1);
tf2::Quaternion current_uncalibrated_rotation_feedback(0, 0, 0, 1);
tf2::Quaternion last_calibrated_rotation_desired;
tf2::Quaternion start_quat_desired;
tf2::Quaternion rotation_quaternion;
ros::Publisher pub;
bool saved_feedback = false;
bool saved_offset = false;


using namespace std;
int main(int argc, char **argv)
{

    // ROS Setup
    ros::init(argc, argv, "feedback_compare");
    ros::NodeHandle handle;
    pub = handle.advertise<geometry_msgs::Quaternion>("comparison", 1);

    ros::Subscriber sub_imu = handle.subscribe("imu_data", 1, imuDataCallback);
		ros::Subscriber sub_feedback = handle.subscribe("feedback_data", 1, feedbackDataCallback);
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
    current_uncalibrated_rotation_desired = new_rotation;
    //new_rotation = quat_offset * new_rotation;
    current_calibrated_rotation_desired = new_rotation;

	if (saved_offset == false && saved_feedback == true) // this compares two of the first desired and feedback values to remove their fixed bias due to misalignment
	{
		saved_offset = true;
		tf2::Quaternion desired_inverse_bias = current_calibrated_rotation_desired.inverse();
		quat_bais = desired_inverse_bias * current_calibrated_rotation_feedback;
    printQuat(&quat_bais);
    printQuat(&current_calibrated_rotation_desired);
    printQuat(&current_calibrated_rotation_feedback);
    tf2::Quaternion should_be_feedback = current_calibrated_rotation_desired * quat_bais;
    printQuat(&should_be_feedback); // this should be equal to the feedback (directly above)
	}

		//compare the two quaternions
		tf2::Quaternion desired_inverse = current_calibrated_rotation_desired.inverse();
    //printQuat(&desired_inverse);
		tf2::Quaternion rotation_between_quats;
    rotation_between_quats = desired_inverse * current_calibrated_rotation_feedback;
		rotation_between_quats = quat_bais * rotation_between_quats; // account for the difference between the starting positions
    //printQuat(&rotation_between_quats);
		
    // Now create the message and publish it
    geometry_msgs::Quaternion output = tf2::toMsg(rotation_between_quats);

    pub.publish(output);
    //printQuat(&rotation_between_quats);

}

void feedbackDataCallback(const geometry_msgs::Quaternion::ConstPtr &msg)
{
	tf2::Quaternion new_rotation;
  tf2::fromMsg(*msg, new_rotation);
  current_uncalibrated_rotation_feedback = new_rotation;
  //new_rotation = quat_offset * new_rotation;
  current_calibrated_rotation_feedback = new_rotation;
	saved_feedback = true;

}

/**
 * Convenience function for debugging
 */
void printQuat(tf2::Quaternion *quat)
{
    std::cout << "Quaternion: (" << quat->getX() << ", " << quat->getY() << ", " << quat->getZ() << ", " << quat->getW() << ")" << std::endl;
}