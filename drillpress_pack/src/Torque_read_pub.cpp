
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h" // Uses Dynamixel SDK library

#include "ros/ros.h"
#include <geometry_msgs/Point.h>

// Control table address
#define ADDR_MX_TORQUE_ENABLE 24 //Control table address is different in Dynamixel model
#define ADDR_MX_PRESENT_LOAD 40
#define ADDR_MOVING_SPEED 32
#define ADDR_MX_PRESENT_POSITION 36
#define ADDR_MX_GOAL_POSITION 30

// Protocol version
#define PROTOCOL_VERSION 1.0 //See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID 2 // Dynamixel ID: 1
#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyUSB0" // Check which port is being used on your controller
#define TORQUE_ENABLE 1           // Value for enabling the torque
#define TORQUE_DISABLE 0          // Value for disabling the torque

#define ESC_ASCII_VALUE 0x1b

double TorqueValue;

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "Torque_Gantry");
  ros::NodeHandle n;
  ros::Publisher Torque_read_pub = n.advertise<geometry_msgs::Point>("Torque_pub", 100);

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL; // Communication result

  uint8_t dxl_error = 0;         // Dynamixel error
  uint16_t dxl_present_load = 0; // Present position
  uint16_t dxl_present_position = 0; // Present position
  // Open port
  portHandler->openPort();

  // Set port baudrate
  portHandler->setBaudRate(BAUDRATE);

  // Enable Dynamixel Torque
  packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  int input =0;

  int spd2 = 50;
  packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MOVING_SPEED, spd2, &dxl_error);

  //printf("input position \n");
  //scanf("%d",&input);
  // Torque Reading thread
  while (ros::ok())
  {
    
    // geometry_msgs::Point torque_read;
    // Read present Torque
    packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_LOAD, &dxl_present_load, &dxl_error);
    // packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    
    // Write Goal position
    printf("input position \n");
    scanf("%d",&input);
    packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, input, &dxl_error);

    // if (dxl_present_load >= 1024)
    // {
    //   //Load works on CW direction
    //   dxl_present_load = dxl_present_load - 1024;
    // }
    // else
    // {
    //   //Load works on CCW direction
    //   dxl_present_load = dxl_present_load;
    // }
    // double present_load = dxl_present_load;
    // TorqueValue = present_load / 1024 * 2.2;

    printf("Present Applied Torque %d \n", dxl_present_load);

    // torque_read.x = dxl_present_load;
    // Torque_read_pub.publish(torque_read);

  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Close port
  portHandler->closePort();

  return 0;
}
