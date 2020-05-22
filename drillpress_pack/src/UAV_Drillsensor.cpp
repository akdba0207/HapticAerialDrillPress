//Author : Dongbin Kim
//Date : 2020-01-17
//Title : Gantry Z axis control with Drillpress encoder input

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define STDIN_FILENO 0
#endif
#include <thread>
#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "dynamixel_sdk.h"      // Uses Dynamixel SDK library
#include <geometry_msgs/Pose.h> // Receive Drill-press input via Point

// Control table address
#define ADDR_MX_TORQUE_ENABLE 24
#define ADDR_MOVING_SPEED 32
#define ADDR_CCW_LIMIT_ANGLE 8
#define ADDR_MX_PRESENT_LOAD 40
#define ADDR_MX_GOAL_POSITION 30

// Protocol version
#define PROTOCOL_VERSION 1.0 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL2_ID 2 // Dynamixel Torque motor ID

#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyUSB2" // Check which port is being used on your controller
#define TORQUE_ENABLE 1           // Value for enabling the torque
#define TORQUE_DISABLE 0          // Value for disabling the torque

// Message parameters
double pos_drill_to_gantry;
double pos_input_raw = 0;
double pos_input;
int pos_z;

//Low Pass Filter
double tau = 0.5;  //time constant
double ts = 0.05;  //sampling time
double prev_y = 0; //previous filtered value

//Motor angle and offset
int dxl2_pose;
int dxl2_input;
double force_offset;
double force_offset_prev;

void offset_motorangle()
{
    while (ros::ok())
    {
        printf("Motor angle release 1, lock 2 \n");
        scanf("%d", &dxl2_input);
        dxl2_pose = dxl2_input * 1000;
        printf("Force Offset \n");
        scanf("%lf", &force_offset);
        force_offset = force_offset + force_offset_prev;
        force_offset_prev = force_offset;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "UAV_Drill");
    ros::NodeHandle nh;
    ros::Publisher gantry_pub = nh.advertise<geometry_msgs::Pose>("torque_to_drill", 100);

    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    uint8_t dxl_error = 0; // Dynamixel error

    // Open port
    portHandler->openPort();

    // Set port baudrate
    portHandler->setBaudRate(BAUDRATE);

    // Enable Dynamixels Torque
    packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

    double present_load2 = 0;
    double TorqueValue2 = 0;
    double applied_force2 = 0;

    uint16_t dxl2_present_load = 0; // Present Load

    geometry_msgs::Pose gantry_read;
    ros::Rate rate(30.0);

    //Initial setup
    int spd2 = 50;
    dxl2_pose = 1000;
    force_offset = 0;
    force_offset_prev = 0;

    packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_MOVING_SPEED, spd2, &dxl_error);

    std::thread t1(offset_motorangle);

    while (ros::ok())
    {
        
        packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl2_pose, &dxl_error);
        //Torque Transmit to Gantry
        packetHandler->read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_LOAD, &dxl2_present_load, &dxl_error);

        double pure_load2 = dxl2_present_load;

        //DXL2 Torque Calculation
        if (pure_load2 >= 1024) //Negative Force sensing case
        {
            //Load works on CW direction
            present_load2 = pure_load2 - 1024;
            TorqueValue2 = 0.00433 * present_load2 - 0.12931;

            if (TorqueValue2 < 0)
            {
                TorqueValue2 = 0;
                applied_force2 = -TorqueValue2 / 0.05;
            }
            else
            {
                applied_force2 = -TorqueValue2 / 0.05;
            }
        }
        else if (pure_load2 > 1020) //Sometimes dynamixel load sensing value stops at 1020~1023 without applied load, therefore I filtered it out.
        {
            pure_load2 = 1020;
            present_load2 = pure_load2;
            TorqueValue2 = 0.00433 * present_load2 - 0.12931;
            applied_force2 = TorqueValue2 / 0.05;
        }
        else //Positive Force sensing case
        {
            //Load works on CCW direction
            present_load2 = pure_load2;
            TorqueValue2 = 0.00433 * present_load2 - 0.12931;

            if (TorqueValue2 < 0)
            {
                TorqueValue2 = 0;
                applied_force2 = TorqueValue2 / 0.05;
            }
            else
            {
                applied_force2 = TorqueValue2 / 0.05;
            }
        }

        //Low pass filter
        double filtered_force = (tau * prev_y + applied_force2 * ts) / (tau + ts);
        prev_y = filtered_force;

        //printf("DXL2 Torque %f (Nm) %f, Applied force %f (N) \n", TorqueValue2, present_load2, filtered_force);

        gantry_read.position.x = TorqueValue2;
        gantry_read.position.y = filtered_force; //For plotting at rqt_plot
        gantry_read.position.z = force_offset;
        gantry_pub.publish(gantry_read);
        rate.sleep();

        ros::spinOnce();
    }

    // Disable Dynamixel#4 Torque
    packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    return 0;
}