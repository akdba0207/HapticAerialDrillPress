//Author : Dongbin Kim
//Date : 2020-02-17
//Title : Drill press control publisher(encoder)/subscriber(the torque input)

//Basic Drill parameters
//Gear 1 : 0.02(m) on dynamixel (MX-28)
//Gear 2 : 0.06(m) on Drill rotary

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "dynamixel_sdk.h" // Uses Dynamixel SDK library

// Control table address
#define ADDR_MX_TORQUE_ENABLE 24 //Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION 30
#define ADDR_MX_PRESENT_POSITION 36
#define ADDR_MX_TORQUE_LIMIT 34
#define ADDR_MX_PRESENT_LOAD 40

// Protocol version
#define PROTOCOL_VERSION 1.0 //See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID 1 // Drill press Dynamixel ID: 1
#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyUSB1" // Check which port is being used on your controller
#define TORQUE_ENABLE 1           // Value for enabling the torque
#define TORQUE_DISABLE 0          // Value for disabling the torque

// Message parameters
double rendered_force;
double rendered_force_raw;
double torque_convert;
double torque_on_drill;
int torque_input;
double force_offset;

void drillpress_torque(const geometry_msgs::Pose msg) //message callback function
{
  rendered_force = msg.position.y;
  force_offset = msg.position.z;
}

int main(int argc, char *argv[])
{
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  uint8_t dxl_error = 0;             // Dynamixel error
  uint16_t dxl_present_position = 0; // Present position
  uint16_t dxl_present_load = 0;     // Present load

  // Open port
  portHandler->openPort();

  // Set port baudrate
  portHandler->setBaudRate(BAUDRATE);

  // Enable Dynamixel Torque
  packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  // Initial Parameters
  int initial_torque;
  int sensitivity;
  int initial_position = 11000;

  int CW_torque_refpose = 2000;
  int CCW_torque_refpose = 11000;
  int torque_refpose;

  // Measurement and changing parameters

  double operator_force = 0; // Applied force on Drillpress rotary.
  double operator_torque = 0;

  double rendered_torque = 0;

  double mapped_force = 0;
  double mapped_torque = 0;

  //ROS node creation
  ros::init(argc, argv, "Drillpress_pubsub");
  ros::NodeHandle n;
  ros::Publisher drill_pub = n.advertise<geometry_msgs::Pose>("drill_to_z", 100);
  ros::Subscriber gantry_sub = n.subscribe("torque_to_drill", 100, drillpress_torque);

  geometry_msgs::Pose drill_z_pub;

  printf("Torque setup \n");
  scanf("%d", &initial_torque);
  packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_LIMIT, initial_torque, &dxl_error);
  packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, initial_position, &dxl_error);
  sleep(1);

  printf("Sensitivity : \n");
  scanf("%d", &sensitivity);
  sleep(1);

  // 30Hz control loop
  ros::Rate rate(30.0);

  while (ros::ok())
  {

    //Offset applied
    rendered_force_raw = (rendered_force - force_offset);
    rendered_force = sensitivity * rendered_force_raw;

    if (rendered_force < 0)
    {
      rendered_torque = -rendered_force * 0.05;
      torque_refpose = CCW_torque_refpose;
    }
    else
    {
      rendered_torque = rendered_force * 0.05;
      torque_refpose = CW_torque_refpose;
    }

    packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, torque_refpose, &dxl_error);

    torque_convert = rendered_torque * 2 / 5;
    torque_on_drill = (torque_convert + 0.12931) / 0.00433; //Drill press input value
    torque_input = abs(torque_on_drill);

    packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_LIMIT, torque_input, &dxl_error);

    // Read present position
    packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_LOAD, &dxl_present_load, &dxl_error);

    double operator_load = dxl_present_load;
    double mapped_load = torque_input;

    ////////////////////////////DXL on drill, Mapped torque and force on the drillpres
    if (rendered_force <= 0)
    {
      mapped_torque = 0.00433 * mapped_load - 0.12931;

      if (mapped_torque < 0)
      {
        mapped_torque = 0;
        mapped_force = mapped_torque / 0.02;
      }
      else
      {
        mapped_force = mapped_torque / 0.02;
      }
    }
    else if (rendered_force > 0)
    {
      mapped_torque = 0.00433 * mapped_load - 0.12931;

      if (mapped_torque < 0)
      {
        mapped_torque = 0;
        mapped_force = -mapped_torque / 0.02;
      }
      else
      {
        mapped_force = -mapped_torque / 0.02;
      }
    }

    ///////////////////////////DXL on drill, Operator Applied force Measurement.
    if (operator_load >= 1024)
    {
      //Load works on CW direction
      operator_load = operator_load - 1024;
      operator_torque = 0.00433 * operator_load - 0.12931;

      if (operator_torque < 0)
      {
        operator_torque = 0;
        operator_force = -operator_torque / 0.02;
      }
      else
      {
        operator_force = -operator_torque / 0.02;
      }
    }
    else if (operator_load > 1020) //Sometimes dynamixel load sensing value stops at 1020~1023 without applied load, therefore I filtered it out.
    {
      operator_load = 1020;
      operator_torque = 0.00433 * operator_load - 0.12931;
      operator_force = operator_torque / 0.02;
    }
    else
    {
      //Load works on CCW direction
      operator_torque = 0.00433 * operator_load - 0.12931;

      if (operator_torque < 0)
      {
        operator_torque = 0;
        operator_force = operator_torque / 0.02;
      }
      else
      {
        operator_force = operator_torque / 0.02;
      }
    }

    printf("D_Pos %d / Rdr_F %lf / Optr_F %lf / Raw %lf / offs %lf / \n", dxl_present_position, mapped_force, operator_force, rendered_force_raw, force_offset);

    double present_position = dxl_present_position;
    drill_z_pub.position.x = present_position;
    drill_z_pub.position.y = mapped_force;          //Rendered Force, plotting at rqt_plot
    drill_z_pub.position.z = operator_force;        //Force input on the drill press by operator For plotting at rqt_plot
    drill_z_pub.orientation.x = rendered_force_raw; //Without sensitivity

    drill_pub.publish(drill_z_pub);
    rate.sleep();
    ros::spinOnce();
  }

  // Disable Dynamixel Torque
  packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

  // Close port
  portHandler->closePort();

  return 0;
}
