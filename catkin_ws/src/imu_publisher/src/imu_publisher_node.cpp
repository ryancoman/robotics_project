// serialstream.h from https://github.com/fedetft/serial-port/tree/master/6_stream
#include "imu_publisher/serialstream.h"
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
// To get the following header working: sudo apt install ros-melodic-tf2-geometry-msgs
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <string>
#include <iostream>

/* The serial port of the IMU device: this will probably change once on the actual computer */
#define DEVICE "/dev/ttyS1"
/* Baud rate for the serial connection */
#define BAUD_RATE 115200

double degToRad(double angle_degrees);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_publisher");

    ros::NodeHandle handle;

    ros::Publisher pub = handle.advertise<geometry_msgs::Quaternion>("imu_data", 10);

    // Set up serial input
    SerialOptions options;
    options.setDevice(DEVICE);
    options.setBaudrate(BAUD_RATE);
    SerialStream serial(options);

    std::string line = "";

    // Skip past bluetooth message
    do
    {
        getline(serial, line);
    } while (std::count(line.begin(), line.end(), '/') == 7);

    while (ros::ok() && getline(serial, line))
    {
        // This will loop indefinitely as long as everything is ok
        // Get each line of the file, then convert Euler angles to Quaternion and publish
        std::stringstream str_stream(line);
        std::string discard;

        double x, y, z;

        // Skip time diff
        std::getline(str_stream, discard, '/');

        // Get x, y, z values
        str_stream >> x;
        str_stream.ignore(1, '/');
        str_stream >> y;
        str_stream.ignore(1, '/');
        str_stream >> z;

        // Convert to quaternion
        tf2::Quaternion output;
        output.setEuler(degToRad(x), degToRad(y), degToRad(z));

        // Now create the message and publish it
        geometry_msgs::Quaternion msg = tf2::toMsg(output);

        pub.publish(msg);
    }

    return EXIT_SUCCESS;
}

/**
 * Convert angle from degrees to radians so it can be used with the setEuler funcction
 */
double degToRad(double angle_degrees)
{
    return angle_degrees / 180 * M_PI;
}