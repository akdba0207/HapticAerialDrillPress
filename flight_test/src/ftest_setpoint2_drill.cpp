//@Copyright : Dongbin Kim.
//Email : dongbin.kim@unlv.edu
//Author : Dongbin Kim
//Title : IROS 2020, Haptic drillpress controlled Mobile Manipulatine UAV with PID compensator
// With Waypoints
//Date : 2020-01-28

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <thread>
#include <iostream>

// Message parameters
double pos_drill_to_uav;
double pos_input_raw = 0;
double pos_input;
double pos_z_input;

//Path planning parameters
double target_x1;
double target_y1;

double target_x2;
double target_y2;

double target_x3;
double target_y3;

double target_x;
double target_y;
double target_z;

double target_x_prev;
double target_y_prev;

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

    pos_drill_to_uav = (msg.position.x - 2050) / (10900 - 2050) * 1.2; //Drill press input value
    pos_z_input = 1.1 + pos_drill_to_uav;
}

void positiontesting()
{
    int mode;

    //Waypoint 1~3, each translation is set to run for 3 seconds
    target_x1 = -1.618; //Initial Point (Stationary)
    target_y1 = -0.039;

    target_x2 = -0.111193; //Acrylic = 0.02, PVC pipe = -0.025939(y-axis parallel), -0.056113 (x-axis parallel),
                            //Wood = -0.041193
    target_y2 = -0.020375; //Acrylic = -0.10, PVC pipe = -0.081550(y-axis parallel), -0.024363 (x-axis parallel)
                            //Wood = -0.100375

    //Target Z was manually controlled, but the height of materials were
    //Acrylic = 0.81, PVC pipe = 0.923558, Wood = 0.824083

    target_x3 = -1.2; //Landing Point (Stationary)
    target_y3 = 0.5;

    ros::Rate rate_trjectory(60.0);
    while (ros::ok())
    {
        printf("Put your mode \n");
        scanf("%d", &mode);
        if (mode == 1)
        {
            target_x = target_x1;
            target_y = target_y1;
        }
        else if (mode == 2)
        {
            //target_x = 0.02;
            //target_y = -0.10;
            for (int i = 1; i < 181; i++)
            {
                target_x = target_x_prev + (target_x2 - target_x_prev) * i / 180;
                target_y = target_y_prev + (target_y2 - target_y_prev) * i / 180;

                rate_trjectory.sleep();
                printf("x = %lf, y = %lf \n", target_x, target_y);
            }
        }
        else if (mode == 3)
        {
            //target_x = -1.2;
            //target_y = 0.5;
            for (int i = 1; i < 181; i++)
            {
                target_x = target_x_prev + (target_x3 - target_x_prev) * i / 180;
                target_y = target_y_prev + (target_y3 - target_y_prev) * i / 180;

                rate_trjectory.sleep();
                printf("x = %lf, y = %lf \n", target_x, target_y);
            }
        }
        target_x_prev = target_x;
        target_y_prev = target_y;
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Setpoint_Node");
    ros::NodeHandle nh;
    ros::Publisher set_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("Desired_set", 10);
    ros::Subscriber drill_pos_sub = nh.subscribe("drill_to_z", 100, drillpress_input);

    ros::Rate rate(60.0);

    geometry_msgs::PoseStamped set_pose;

    std::thread t1(positiontesting);
    while (ros::ok())
    {

        set_pose.pose.position.x = target_x;
        set_pose.pose.position.y = target_y;
        set_pose.pose.position.z = pos_z_input; //Drillpress input, comment if you are not using.

        //For tuning purpose, drill press input is not used
        // set_pose.pose.position.z = 1.5;

        set_pose.pose.orientation.x = 0;
        set_pose.pose.orientation.y = 0;
        set_pose.pose.orientation.z = 0;

        set_pos_pub.publish(set_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}