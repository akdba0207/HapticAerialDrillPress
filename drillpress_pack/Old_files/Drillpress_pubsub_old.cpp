//Author : Dongbin Kim
//Date : 2020-01-11
//Title : Drill press control publisher(encoder)/subscriber(the torque input)

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include "dynamixel_sdk.h" // Uses Dynamixel SDK library

// Control table address
#define ADDR_MX_TORQUE_ENABLE 24 //Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION 30
#define ADDR_MX_PRESENT_POSITION 36
#define ADDR_MX_TORQUE_LIMIT 34

// Protocol version
#define PROTOCOL_VERSION 1.0 //See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID 1 // Drill press Dynamixel ID: 1
#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyUSB0" // Check which port is being used on your controller
#define TORQUE_ENABLE 1           // Value for enabling the torque
#define TORQUE_DISABLE 0          // Value for disabling the torque

int main(int argc, char *argv[])
{
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  uint8_t dxl_error = 0;             // Dynamixel error
  uint16_t dxl_present_position = 0; // Present position
  uint16_t dxl_previous_position = 0;

  // Open port
  portHandler->openPort();

  // Set port baudrate
  portHandler->setBaudRate(BAUDRATE);

  // Enable Dynamixel Torque
  packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  // Previous Dynamixel Position initial setup.
  packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_previous_position, &dxl_error);
  double count = 0;
  int TorqueInput;

  //ROS node creation
  ros::init(argc, argv, "Drillpress_pubsub");
  ros::NodeHandle n;
  ros::Publisher drill_pub = n.advertise<geometry_msgs::Point>("drill_to_z", 100);

  geometry_msgs::Point drill_z_pub;

  printf("Torque setup \n");
  scanf("%d", &TorqueInput);

  ros::Rate rate(30.0);

  while (ros::ok())
  {

    packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_LIMIT, TorqueInput, &dxl_error);

    // Read present position
    packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    if ((dxl_previous_position - dxl_present_position) > 10)
    {
      count = -1;
      dxl_previous_position = dxl_present_position;
    }
    else if ((dxl_previous_position - dxl_present_position) < -3000)
    {
      count = -1;
      dxl_previous_position = dxl_present_position;
    }
    else if ((dxl_previous_position - dxl_present_position) < -10)
    {
      count = 1;
      dxl_previous_position = dxl_present_position;
    }
    else if ((dxl_previous_position - dxl_present_position) > 3000)
    {
      count = 1;
      dxl_previous_position = dxl_present_position;
    }
    else if ((dxl_previous_position - dxl_present_position) >= -10)
    {
      count = 0;
      dxl_previous_position = dxl_present_position;
    }
    else if ((dxl_previous_position - dxl_present_position) <= 10)
    {
      count = 0;
      dxl_previous_position = dxl_present_position;
    }

    printf("Present Pos %d Previous Pos %d Count %f\n", dxl_present_position, dxl_previous_position, count);

    drill_z_pub.x = count;

    drill_pub.publish(drill_z_pub);
    rate.sleep();
  }

  // Disable Dynamixel Torque
  packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

  // Close port
  portHandler->closePort();

  return 0;
}
