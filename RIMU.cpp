/*
 * To use this, you will need the boost libraries.
 * To install on a debian/ubuntu linux system, I used 'sudo apt install libboost-all-dev'
 * 18.04
 */
// serialstream.h from https://github.com/fedetft/serial-port/tree/master/6_stream
#include "serialstream.h"
#include <iostream>
#include <boost/math/quaternion.hpp>
#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <string>

// Constants //
/* The serial port of the IMU device: this will probably change once on the actual computer */
#define DEVICE "/dev/ttyS1"
/* Baud rate for the serial connection */
#define BAUD_RATE 115200

using namespace boost::math;

SerialStream connectToIMU(std::string device, int baudRate);

quaternion<double> eulerToQuaternion(double x, double y, double z);

quaternion<double> parseImuData(std::string line);

int main()
{
    std::cout << "BEGIN" << std::endl;

    quaternion<double> quat_offset(0, 0, 0, 1);
    quaternion<double> start_quat = eulerToQuaternion(180, 0, -30);
    quaternion<double> current_uncalibrated_rotation(0, 0, 0, 1);
    quaternion<double> current_calibrated_rotation = new Quaternion(0, 0, 0, 1);

    SerialStream serial = connectToIMU(DEVICE, BAUD_RATE);

    std::string line;

    while (true) // This will loop forever, so the rest of the code is unreachable. Perhaps there's a more elegant way to handle this?
    {
        // The original code just queued up the data and then dumped it if there was too much.
        // This will process each element individually
        getline(serial, line);
    }

    // Note: unreachable code
    std::cout << "END" << std::endl;
    return EXIT_SUCCESS;
}

/**
 * Connect to the IMU and skip the BT message at the beginning
 *
 * @param device the name of the device, for example
 */
SerialStream connectToIMU(std::string device, int baudRate)
{
    SerialOptions options;
    options.setDevice(DEVICE);
    options.setBaudrate(BAUDRATE);
    SerialStream serial(options);

    std::string line = "";

    do
    {
        getline(serial, line);
    } while (std::count(line.begin(), line.end(), '/') == 7);

    return serial;
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

/**
 * Parses the quaternion from a single line from the IMU and returns the quaternion
 */
quaternion<double> parseImuData(std::string line)
{
}