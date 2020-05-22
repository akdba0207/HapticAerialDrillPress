//@Copyright : Dongbin Kim.
//Email : dongbin.kim@unlv.edu
//Date : 2020-01-05
//Title : Using Customized haptic device to control drone position in SITL

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <thread>
#include <iostream>

// Message parameters
double pos_drill_to_uav;
double pos_input_raw = 0;
double pos_input;
double pos_z_input;


mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

struct UAV_State
{
    double uavpose[3];
    double uavorient[3];
};

class Drillpress_Setpoint
{
public:
    ros::NodeHandle n;
    ros::Publisher Set_Pub;
    ros::Subscriber Drillpress_sub;
    UAV_State *state;

    void init(UAV_State *s)
    {
        Set_Pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        Drillpress_sub = n.subscribe<geometry_msgs::Pose>("drill_to_z", 100, &Drillpress_Setpoint::drillpress_input, this);
        state = s;
    }

    
    void setpoint_publish()
    {
        //Test and Evaluation platform in mocap area information
        //Target material height = 0.8 (m)

        geometry_msgs::PoseStamped setpoint_pub;

        setpoint_pub.pose.position.x = state->uavpose[0];
        setpoint_pub.pose.position.y = state->uavpose[1];
        setpoint_pub.pose.position.z = state->uavpose[2];

        setpoint_pub.pose.orientation.x = state->uavorient[0] * 0;
        setpoint_pub.pose.orientation.y = state->uavorient[1] * 0;
        setpoint_pub.pose.orientation.z = state->uavorient[2] * 0;

        Set_Pub.publish(setpoint_pub);
    }

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
        
        state->uavpose[0] = 0;
        state->uavpose[1] = 0;
        state->uavpose[2] = pos_z_input;

        state->uavorient[0] = 0;
        state->uavorient[1] = 0;
        state->uavorient[2] = 0;

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    UAV_State state;
    Drillpress_Setpoint dp_setpoint_ros;

    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    dp_setpoint_ros.init(&state);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

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

        dp_setpoint_ros.setpoint_publish();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}