#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
// To get the following header working: sudo apt install ros-melodic-tf2-geometry-msgs
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <fstream>

void comparisionCallback(const geometry_msgs::Quaternion::ConstPtr &msg);
void printQuat(tf2::Quaternion *quat);
using namespace std;
ofstream outdata; // outdata is like cin

void closeAll(void)
{
	outdata.close();
}



int main(int argc, char **argv)
{
		atexit(closeAll);

    // ROS Setup
    ros::init(argc, argv, "process_data");
    ros::NodeHandle handle;
    //pub = handle.advertise<geometry_msgs::Pose>("desired_arm_pose", 1);

    ros::Subscriber sub = handle.subscribe("comparison", 1, comparisionCallback);
		
		outdata.open("Comparision_Quaterion_actual0.xlsx"); // opens the file
   	if( !outdata ) { // file couldn't be opened
      cerr << "Error: file could not be opened" << endl;
      exit(1);
   	}

    // According to this (https://answers.ros.org/question/208587/shared-variable-between-callbacks/) page,
    // using ros::spin(); with global variables IS safe because only a single thread handles callbacks.
    // Therefore, in imuDataCallback() it is fine to use and modify globals
    ros::spin();
}

/**
*Writes the given quaterion out to the excel file to save
*/
void comparisionCallback(const geometry_msgs::Quaternion::ConstPtr &msg)
{
  tf2::Quaternion quat;
  tf2::fromMsg(*msg, quat);
  outdata << "Quaternion: (" << quat.getX() << ", " << quat.getY() << ", " << quat.getZ() << ", " << quat.getW() << ")" << std::endl;
  //printQuat(&quat);
}

/**
 * Convenience function for debugging
 */
void printQuat(tf2::Quaternion *quat)
{
    std::cout << "Quaternion: (" << quat->getX() << ", " << quat->getY() << ", " << quat->getZ() << ", " << quat->getW() << ")" << std::endl;
}