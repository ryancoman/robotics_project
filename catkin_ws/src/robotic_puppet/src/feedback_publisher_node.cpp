// serialstream.h from https://github.com/fedetft/serial-port/tree/master/6_stream
#include "robotic_puppet/serialstream.h"
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
// To get the following header working: sudo apt install ros-melodic-tf2-geometry-msgs
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <string>
#include <iostream>

/* The serial port of the IMU device: this will probably change once on the actual computer */
#define DEVICE "/dev/ttyUSB1"
// #define DEVICE "/dev/ttyS1"
/* Baud rate for the serial connection */
#define BAUD_RATE 115200

double degToRad(double angle_degrees);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feedback_node_publisher");

    ros::NodeHandle handle;

    ros::Publisher pub = handle.advertise<geometry_msgs::Quaternion>("feedback_data", 10);

    // Set up serial input
    SerialOptions options;
    options.setDevice(DEVICE);
    options.setBaudrate(BAUD_RATE);
    // options.setTimeout(boost::posix_time::seconds(3));
    SerialStream serial(options);

    std::string line = "";
    std::cout << "Preparing to read..." << std::endl;
    // Skip past bluetooth message
    do
    {
        getline(serial, line);
        std::cout << line << std::endl;
    } while (std::count(line.begin(), line.end(), '/') != 8); // Format for proper line: dt/w/x/y/z/ax/ay/az/
    std::cout << "Reading data feedback " << std::endl;
    while (ros::ok())
    {
        std::cout << "feedback should be reading data now" << std::endl;
        // This will loop indefinitely as long as everything is ok
        // Get each line of the file, then convert Euler angles to Quaternion and publish
        if (!getline(serial, line))
        {
            std::cout << "line not found" << std::endl;
            continue; // skip past to reread data
        }

        if (std::count(line.begin(), line.end(), '/') != 8)
        {
            // Skip line because no quaternion data included
            continue;
        }

        std::stringstream str_stream(line);
        std::string discard;

        double w, x, y, z;

        // Skip time diff
        std::getline(str_stream, discard, '/');

        // Get x, y, z values
        str_stream >> w;
        str_stream.ignore(1, '/');
        str_stream >> x;
        str_stream.ignore(1, '/');
        str_stream >> y;
        str_stream.ignore(1, '/');
        str_stream >> z;

        // Convert to quaternion
        tf2::Quaternion output(x, y, z, w);
        // output.setEuler(degToRad(x), degToRad(y), degToRad(z));

        // Now create the message and publish it
        geometry_msgs::Quaternion msg = tf2::toMsg(output);

        pub.publish(msg);
    }
    std::cout << "Exiting..." << std::endl;
    return EXIT_SUCCESS;
}

/**
 * Convert angle from degrees to radians so it can be used with the setEuler funcction
 */
double degToRad(double angle_degrees)
{
    return angle_degrees / 180 * M_PI;
}