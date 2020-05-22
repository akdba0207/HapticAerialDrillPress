//Author : Dongbin Kim
//Date : 2020-01-17
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
#define DXL4_ID 4 // Dynamixel ID: 4
#define DXL2_ID 2 // Dynamixel Torque motor ID

#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyUSB1" // Check which port is being used on your controller
#define TORQUE_ENABLE 1           // Value for enabling the torque
#define TORQUE_DISABLE 0          // Value for disabling the torque

// Message parameters
double pos_drill_to_gantry;
double pos_input_raw =0;
double pos_input;
int pos_z;

//Low Pass Filter
double tau = 0.5;  //time constant
double ts = 0.05;  //sampling time
double prev_y = 0; //previous filtered value

void drillpress_input(const geometry_msgs::Pose msg) //message callback function
{
    //Safety trigger for multi-turn controlled dynamixel on the drill press
    pos_input_raw = msg.position.x;
    if(pos_input_raw < 2050)
    {
        pos_input = 0;
    }
    else if(pos_input_raw > 10200)
    {
        pos_input = 0;
    }
    else
    {
        pos_input= pos_input_raw;
    }
    
    pos_drill_to_gantry = (msg.position.x - 2050) / (10900 - 2050) * 5550; //Drill press input value
    pos_z = 900 - pos_drill_to_gantry;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Gantry_Z");
    ros::NodeHandle nh;
    ros::Subscriber gantry_sub = nh.subscribe("drill_to_z", 100, drillpress_input);
    ros::Publisher gantry_pub = nh.advertise<geometry_msgs::Pose>("torque_to_drill", 100);

    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

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

    double present_load2 = 0;
    double TorqueValue2 = 0;

    double present_load4 = 0;
    double TorqueValue4 = 0;

    double applied_force2 = 0;

    uint16_t dxl2_present_load = 0; // Present Load
    uint16_t dxl4_present_load = 0; // Present Load

    geometry_msgs::Pose gantry_read;
    ros::Rate rate(30.0);

    //Motor speed setup
    int spd4 = 250;
    int spd2 = 50;
    int dxl2_pose;

    packetHandler->write2ByteTxRx(portHandler, DXL4_ID, ADDR_MOVING_SPEED, spd4, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_MOVING_SPEED, spd2, &dxl_error);

    printf("Rotary Drill joint pose : \n");
    scanf("%d", &dxl2_pose);
    packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl2_pose, &dxl_error);

    while (ros::ok())
    {
        // Drill press motor input is from 2194(bottom) to 10900(top)
        // Gantry Z motor value is from 3550(bottom) to -2000(top)
        // Always maintain on multi-turn mode.
        // I set it through dynamixel master software on windows
        // Both motor value should be mapped together.

        //Gantry Z-axis input

        packetHandler->write2ByteTxRx(portHandler, DXL4_ID, ADDR_MX_GOAL_POSITION, pos_z, &dxl_error);

        //Torque Transmit to Gantry
        packetHandler->read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_LOAD, &dxl2_present_load, &dxl_error);
        packetHandler->read2ByteTxRx(portHandler, DXL4_ID, ADDR_MX_PRESENT_LOAD, &dxl4_present_load, &dxl_error);

        double pure_load2 = dxl2_present_load;
        // double pure_load4 = dxl4_present_load;

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

        printf("DXL2 Torque %f (Nm) %f, Applied force %f (N) \n", TorqueValue2, present_load2, filtered_force);

        gantry_read.position.x = TorqueValue2;
        gantry_read.position.y = filtered_force; //For plotting at rqt_plot

        gantry_pub.publish(gantry_read);
        rate.sleep();

        ros::spinOnce();
    }

    // Disable Dynamixel#4 Torque
    packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    return 0;
}