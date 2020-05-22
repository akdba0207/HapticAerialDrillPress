//Author : Dongbin Kim
//Date : 2020-02-06
//Title : Data Arrangement for real-time plotting

#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h> // Receive Drill-press input via Point

// Message parameters
double mapped_force;
double operator_force;
double mapped_force_raw;

void data_input(const geometry_msgs::Pose msg) //message callback function
{
    mapped_force = msg.position.y;
    operator_force = msg.position.z;
    mapped_force_raw = msg.orientation.x;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Data_node");
    ros::NodeHandle nh;
    ros::Subscriber Data_sub = nh.subscribe("drill_to_z", 100, data_input);
    ros::Publisher Data_pub = nh.advertise<geometry_msgs::Pose>("Data_arranged", 100);

    geometry_msgs::Pose data_arranged;
    ros::Rate rate(30.0);

    while (ros::ok())
    {
        double haptic_judge;

        //For expert, to tell if he applies the force weaker, equal, or stronger to the rendered force
        if (mapped_force < 0)
        {
            if (operator_force >= 0)
            {
                double sum = mapped_force + operator_force;
                if (sum < -0.5)
                {
                    //haptic_judge = -10; //Weaker - Can't resist the drill press rotation
                    system("clear");
                    printf("Weak, you need %lf (N) to move the rotary\n", sum);
                }
                else if (sum > 0.5)
                {
                    // haptic_judge = 10; //Strong - Can resist the drill press rotation, rotary is moving
                    system("clear");
                    printf("Strong, you're putting over %lf (N)\n", -sum);
                }
                else
                {
                    // haptic_judge = 0; //Equal - Can resist the drill press rotation, rotary is paused
                    system("clear");
                    printf("The drillpress is on hold\n");
                }
            }
            else
            {
                // haptic_judge = -5;
                system("clear");
                printf("Smooth, Force feedback and Drillpress force input are parallel\n");
            }
        }
        else
        {
            if (operator_force <= 0)
            {
                double sum = mapped_force + operator_force;
                if (sum < -0.5)
                {
                    //  haptic_judge = 10; //Strong - Can resist the drill press rotation, rotary is moving
                    system("clear");
                    printf("Strong, you're putting over %lf (N)\n", -sum);
                }
                else if (sum > 0.5)
                {
                    // haptic_judge = -10; //Weaker - Can't resist the drill press rotation
                    system("clear");
                    printf("Weak, you need %lf (N) to move the rotary\n", sum);
                }
                else
                {
                    //  haptic_judge = 0; //Equal - Can resist the drill press rotation, rotary is paused
                    system("clear");
                    printf("The drillpress is on hold\n");
                }
            }
            else
            {
                // haptic_judge = 5;
                system("clear");
                printf("Smooth, Force feedback and Drillpress force input are parallel\n");
            }
        }

        data_arranged.position.x = -mapped_force;   //Mapped force input on the drillpress
        data_arranged.position.y = -operator_force; //Exerted force input on the drillpress
        //data_arranged.position.z = haptic_judge;     //Judging the status of the haptic feedback
        data_arranged.orientation.x = mapped_force_raw; //What's exerted on the gantry dynamixel

        Data_pub.publish(data_arranged);
        rate.sleep();

        ros::spinOnce();
    }

    return 0;
}