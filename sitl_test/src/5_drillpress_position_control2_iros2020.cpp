//@Copyright : Dongbin Kim.
//Email : dongbin.kim@unlv.edu
//Date : 2020-01-17
//Title : Using Customized haptic device to control drone position in SITL
// Waypoint Added, thus the operator control drone to mimic drillpressing motion.

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <thread>
#include <iostream>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}
double lmao1;
double lmao2;
double lmao3;

// Message parameters
double pos_drill_to_uav;
double pos_input_raw = 0;
double pos_input;
double pos_z_input;
int mode;


void drillpress_input(const geometry_msgs::Pose msg) //message callback function
{
    //Safety trigger for multi-turn controlled dynamixel on the drill press
    pos_input_raw = msg.position.x;
    if (pos_input_raw < 2050)
    {
        pos_input = 0;
    }
    else if (pos_input_raw > 10200)
    {
        pos_input = 0;
    }
    else
    {
        pos_input = pos_input_raw;
    }

    pos_drill_to_uav = (msg.position.x - 2050) / (10900 - 2050) * 5; //Drill press input value
    pos_z_input = 10 + pos_drill_to_uav;
}

void positiontesting()
{
    while (ros::ok())
    {
        // printf("type x\n");
        // scanf("%lf", &lmao1);
        // printf("type z\n");
        // scanf("%lf", &lmao3);
        // printf("type y\n");
        // scanf("%lf", &lmao2);
        printf("Put your mode \n");
        scanf("%d",&mode);
        if(mode ==1){
            lmao1 = 0;
            lmao2 = -5;
            //lmao3 = 5;
        }
        else if(mode==2){
            lmao1 = 5;
            lmao2 = -5;
            //lmao3 = 5;
        }
        else if(mode==3){
            lmao1 = 0;
            lmao2 = 0;
            //lmao3 = pos_z_input;    
        }
               

    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber drill_pos_sub = nh.subscribe("drill_to_z", 100, drillpress_input);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(60.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    std::thread t1(positiontesting);
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
       
        //printf("%f \n",pos_z_input);
        pose.pose.position.x = lmao1;
        pose.pose.position.y = lmao2;
        pose.pose.position.z = pos_z_input;

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}