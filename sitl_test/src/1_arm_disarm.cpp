//@Copyright : Dongbin Kim.
//Email : dongbin.kim@unlv.edu
//Date : 2019-05-21
//Title : Practice Arming and Disarming in SITL

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <unistd.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    

    while(ros::ok()){
        
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        arming_client.call(arm_cmd);

        sleep(5);

        arm_cmd.request.value = false;
        arming_client.call(arm_cmd);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}