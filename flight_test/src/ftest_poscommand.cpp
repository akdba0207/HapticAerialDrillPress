//@Copyright : Dongbin Kim.
//Email : dongbin.kim@unlv.edu

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
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
int mode;
geometry_msgs::PoseStamped pose_data; 

void pidloop(const geometry_msgs::PoseStamped pose_data){
   
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
            lmao2 = -1.2;
            lmao3 = 1.3;
        }
        else if(mode==2){
            lmao1 = 0;
            lmao2 = 0.02;
            lmao3 = 1.3;
        }
        else if(mode==3){
            lmao1 = 0;
            lmao2 = 0.02;
            lmao3 = 1.23;    
        }
        else if(mode==4){
            lmao1 = 0;
            lmao2 = 0.01;
            lmao3 = 1.4;
        }
        else if(mode==5){
            lmao1 = 0;
            lmao2 = -1.2;
            lmao3 = 1.3;
        }

    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    
    ros::Subscriber mc_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/optitrack/mc_to_px4",10, pidloop);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    std::thread t1(positiontesting);
    
    while (ros::ok())
    {
        
        pose.pose.position.x = lmao1;
        pose.pose.position.y = lmao2;
        pose.pose.position.z = lmao3;

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}