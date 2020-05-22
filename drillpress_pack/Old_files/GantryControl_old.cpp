//Author : Dongbin Kim
//Date : 2020-01-11
//Title : Gantry Z axis control with Drillpress encoder input

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define STDIN_FILENO 0
#endif

#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "dynamixel_sdk.h"       // Uses Dynamixel SDK library
#include <geometry_msgs/Point.h> // Receive Drill-press input via Point

// Control table address
#define ADDR_MX_TORQUE_ENABLE 24
#define ADDR_MOVING_SPEED 32
#define ADDR_CCW_LIMIT_ANGLE 8
#define ADDR_MX_PRESENT_LOAD 40

// Protocol version
#define PROTOCOL_VERSION 1.0 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL4_ID 4 // Dynamixel ID: 4
#define DXL2_ID 2 // Dynamixel Torque motor ID

#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyUSB1" // Check which port is being used on your controller
#define TORQUE_ENABLE 1           // Value for enabling the torque
#define TORQUE_DISABLE 0          // Value for disabling the torque

// Message parameters
double pos_z;

void drillpress_input(const geometry_msgs::Point msg) //message callback function
{
    pos_z = msg.x; //Drill press input value
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Gantry_Z");
    ros::NodeHandle nh;
    ros::Subscriber gantry_sub = nh.subscribe("drill_to_z", 100, drillpress_input);
    ros::Publisher gantry_pub = nh.advertise<geometry_msgs::Point>("torque_to_drill", 100);

    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int spd = 50;

    uint8_t dxl_error = 0; // Dynamixel error

    // Open port
    portHandler->openPort();

    // Set port baudrate
    portHandler->setBaudRate(BAUDRATE);

    // Enable Dynamixels Torque
    packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

    //Convert Dynamixel into Wheelmode -> CW and CCW limit to value 0
    packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_CCW_LIMIT_ANGLE, 0, &dxl_error);

    double previous_z = 0;
    double present_z = 0;
    double present_load = 0;
    double TorqueValue = 0;
    uint16_t dxl_present_load = 0; // Present Load

    geometry_msgs::Point torque_read;
    ros::Rate rate(30.0);

    while (ros::ok())
    {
        // present_z = pos_z / 180 * 30;
        // displ_z = present_z - previous_z;
        // if (displ_z < 0)
        // {
        //     dist_z = -displ_z;
        // }
        // else if (displ_z > 0)
        // {
        //     dist_z = displ_z;
        // }
        // Calculation of sleeping time
        // float time_z = (5 * dist_z) / (spd * 0.1445 * 2) * 1000000;
        // int time_z1 = time_z;

        // printf(" %d     %f \n", time_z1, time_z);

        //Moving Gantry Z-axis part
        if (pos_z == 1)
        {
            packetHandler->write2ByteTxRx(portHandler, DXL4_ID, ADDR_MOVING_SPEED, spd + 1024, &dxl_error); //ccw rota\tion
            // usleep(time_z1);
            // packetHandler->write2ByteTxRx(portHandler, DXL4_ID, ADDR_MOVING_SPEED, 0, &dxl_error); //ccw rota\tion
        }
        else if (pos_z == -1)
        {
            packetHandler->write2ByteTxRx(portHandler, DXL4_ID, ADDR_MOVING_SPEED, spd, &dxl_error); //cw rota\tion
            // usleep(time_z1);
            // packetHandler->write2ByteTxRx(portHandler, DXL4_ID, ADDR_MOVING_SPEED, 0, &dxl_error); //cw rota\tion
        }
        else if (pos_z == 0)
        {
            // packetHandler->write2ByteTxRx(portHandler, DXL4_ID, ADDR_MOVING_SPEED, spd, &dxl_error); //cw rota\tion
            // usleep(time_z1);
            packetHandler->write2ByteTxRx(portHandler, DXL4_ID, ADDR_MOVING_SPEED, 0, &dxl_error); //cw rota\tion
        }

        //Torque Transmit to Gantry
        packetHandler->read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_LOAD, &dxl_present_load, &dxl_error);

        double pure_load = dxl_present_load;

        if (pure_load >= 1024)
        {
            //Load works on CW direction
            present_load = pure_load - 1024;
            TorqueValue = -(present_load / 1024 * 2.2);
        }
        else
        {
            //Load works on CCW direction
            present_load = pure_load;
            TorqueValue = present_load / 1024 * 2.2;
        }

        printf("Present Applied Torque %f (Nm) %f \n", TorqueValue, present_load);

        torque_read.x = present_load;
        torque_read.y = pure_load;
        torque_read.z = TorqueValue;

        gantry_pub.publish(torque_read);
        rate.sleep();

        ros::spinOnce();
    }

    // Disable Dynamixel#4 Torque
    packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    return 0;
}