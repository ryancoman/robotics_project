/*
 * To use this, you will need the boost libraries.
 * To install on a debian/ubuntu linux system, I used 'sudo apt install libboost-all-dev'
 */
// serialstream.h from https://github.com/fedetft/serial-port/tree/master/6_stream
#include "imu_publisher/serialstream.h"
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <boost/math/quaternion.hpp>
#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <string>
#include <iostream>

/* The serial port of the IMU device: this will probably change once on the actual computer */
#define DEVICE "/dev/ttyS1"
/* Baud rate for the serial connection */
#define BAUD_RATE 115200

using namespace boost::math;

quaternion<double> eulerToQuaternion(double x, double y, double z);

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
        quaternion<double> output = eulerToQuaternion(x, y, z);

        geometry_msgs::Quaternion msg;
        msg.w = output.R_component_1();
        msg.x = output.R_component_2();
        msg.y = output.R_component_3();
        msg.z = output.R_component_4();

        pub.publish(msg);
    }

    return EXIT_SUCCESS;
}

/**
 * From wikipedia: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * Uses angles in degrees
 */
quaternion<double> eulerToQuaternion(double x, double y, double z)
{
    // Convert all angles to radians first
    x = x / 180 * double_constants::pi;
    y = y / 180 * double_constants::pi;
    z = z / 180 * double_constants::pi;

    // Abbreviations for the various angular functions
    double cx = cos(x * 0.5);
    double sx = sin(x * 0.5);
    double cy = cos(y * 0.5);
    double sy = sin(y * 0.5);
    double cz = cos(z * 0.5);
    double sz = sin(z * 0.5);

    double qw = cx * cy * cz + sx * sy * sz;
    double qx = sx * cy * cz - cx * sy * sz;
    double qy = cx * sy * cz + sx * cy * sz;
    double qz = cx * cy * sz - sx * sy * cz;

    quaternion<double> q(qw, qx, qy, qz);
    return q;
}