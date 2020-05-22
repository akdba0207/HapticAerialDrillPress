//@Copyright : Dongbin Kim.
//Email : dongbin.kim@unlv.edu
//Author : Dongbin Kim
//Title : IROS 2020, Haptic drillpress controlled Mobile Manipulatine UAV with PID compensator
//Date : 2020-01-28


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

//P gain for position and orientation
#define Kpx 2.4
#define Kpy 2.6
#define Kpz 3.0

#define Kpox 0.3
#define Kpoy 0.3
#define Kpoz 0.01

//I gain for position and orientation
#define Kix 0.01
#define Kiy 0.01
#define Kiz 0.01

#define Kiox 0.1
#define Kioy 0.1
#define Kioz 0.01

//D gain for position and orientation
#define Kdx 1.3
#define Kdy 1.3
#define Kdz 1.2

#define Kdox 0.1
#define Kdoy 0.1
#define Kdoz 0.01

struct UAV_State
{
    //Finalized pid feedback and mocap raw data
    double pid_feedback[6];
    double uavpose_mocap[3];
    double uavorient_mocap[3];

    //Position feedback
    double uavpose[3];
    double uavpose_er[3];
    double uavpose_p_er[3];

    double uavpose_p[3];
    double uavpose_i[3];
    double uavpose_d[3];

    //Orientation feedback
    double uavorient[3];
    double uavorient_er[3];
    double uavorient_p_er[3];

    double uavorient_p[3];
    double uavorient_i[3];
    double uavorient_d[3];
};

class Drillpress_Setpoint
{
public:
    ros::NodeHandle n;
    ros::NodeHandle n1;
    ros::Publisher Set_Pub;
    ros::Publisher Data_Pub;
    ros::Subscriber Desire_set;
    ros::Subscriber Mc_pose_sub;
    UAV_State *state;

    void init(UAV_State *s)
    {
        Set_Pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        Data_Pub = n1.advertise<geometry_msgs::Pose>("Data_arranged2", 10);

        Desire_set = n.subscribe<geometry_msgs::PoseStamped>("Desired_set", 100, &Drillpress_Setpoint::set_msgs, this);
        Mc_pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/optitrack/mc_to_px4", 10, &Drillpress_Setpoint::pidloop, this);
        state = s;
    }

    void set_msgs(const geometry_msgs::PoseStamped set_msgs)
    {
        state->uavpose[0] = set_msgs.pose.position.x;
        state->uavpose[1] = set_msgs.pose.position.y;
        state->uavpose[2] = set_msgs.pose.position.z;

        state->uavorient[0] = set_msgs.pose.orientation.x;
        state->uavorient[1] = set_msgs.pose.orientation.y;
        state->uavorient[2] = set_msgs.pose.orientation.z;
    }

    void pidloop(const geometry_msgs::PoseStamped mc_data)
    {
        //Current position from Mocap
        state->uavpose_mocap[0] = mc_data.pose.position.x;
        state->uavpose_mocap[1] = mc_data.pose.position.y;
        state->uavpose_mocap[2] = mc_data.pose.position.z;

        state->uavorient_mocap[0] = mc_data.pose.orientation.x;
        state->uavorient_mocap[1] = mc_data.pose.orientation.y;
        state->uavorient_mocap[2] = mc_data.pose.orientation.z;

        //Current Error
        state->uavpose_er[0] = state->uavpose[0] - state->uavpose_mocap[0];
        state->uavpose_er[1] = state->uavpose[1] - state->uavpose_mocap[1];
        state->uavpose_er[2] = state->uavpose[2] - state->uavpose_mocap[2];

        state->uavorient_er[0] = state->uavorient[0] - state->uavorient_mocap[0];
        state->uavorient_er[1] = state->uavorient[1] - state->uavorient_mocap[1];
        state->uavorient_er[2] = state->uavorient[2] - state->uavorient_mocap[2];

        //P Control
        state->uavpose_p[0] = state->uavpose_er[0] * Kpx;
        state->uavpose_p[1] = state->uavpose_er[1] * Kpy;
        state->uavpose_p[2] = state->uavpose_er[2] * Kpz;

        state->uavorient_p[0] = state->uavorient_er[0] * Kpox;
        state->uavorient_p[1] = state->uavorient_er[1] * Kpoy;
        state->uavorient_p[2] = state->uavorient_er[2] * Kpoz;

        //I Control
        //state->uavpose_i[0] = (state->uavpose_i[0] + (state->uavpose_er[0]) * 0.01) * Kix;
        //state->uavpose_i[1] = (state->uavpose_i[1] + (state->uavpose_er[1]) * 0.01) * Kiy;
        //state->uavpose_i[2] = (state->uavpose_i[2] + (state->uavpose_er[2]) * 0.01) * Kiz;
        state->uavpose_i[0] = 0;
        state->uavpose_i[1] = 0;
        state->uavpose_i[2] = 0;

        state->uavorient_i[0] = 0;
        state->uavorient_i[1] = 0;
        state->uavorient_i[2] = 0;

        //D Control
        state->uavpose_d[0] = (state->uavpose_er[0] - state->uavpose_p_er[0]) * Kdx / 0.01;
        state->uavpose_d[1] = (state->uavpose_er[1] - state->uavpose_p_er[1]) * Kdy / 0.01;
        state->uavpose_d[2] = (state->uavpose_er[2] - state->uavpose_p_er[2]) * Kdz / 0.01;

        state->uavorient_d[0] = (state->uavorient_er[0] - state->uavorient_p_er[0]) * Kdox / 0.01;
        state->uavorient_d[1] = (state->uavorient_er[1] - state->uavorient_p_er[1]) * Kdox / 0.01;
        state->uavorient_d[2] = (state->uavorient_er[2] - state->uavorient_p_er[2]) * Kdox / 0.01;

        //Previous Error
        state->uavpose_p_er[0] = state->uavpose_er[0];
        state->uavpose_p_er[1] = state->uavpose_er[1];
        state->uavpose_p_er[2] = state->uavpose_er[2];

        state->uavorient_p_er[0] = state->uavorient_er[0];
        state->uavorient_p_er[1] = state->uavorient_er[1];
        state->uavorient_p_er[2] = state->uavorient_er[2];

        //Final Feedback
        state->pid_feedback[0] = state->uavpose_p[0] + state->uavpose_i[0] + state->uavpose_d[0];
        state->pid_feedback[1] = state->uavpose_p[1] + state->uavpose_i[1] + state->uavpose_d[1];
        state->pid_feedback[2] = state->uavpose_p[2] + state->uavpose_i[2] + state->uavpose_d[2];

        state->pid_feedback[3] = state->uavorient_p[0] + state->uavorient_i[0] + state->uavorient_d[0];
        state->pid_feedback[4] = state->uavorient_p[1] + state->uavorient_i[1] + state->uavorient_d[1];
        state->pid_feedback[5] = state->uavorient_p[2] + state->uavorient_i[2] + state->uavorient_d[2];
    }
    void setpoint_publish()
    {

        geometry_msgs::PoseStamped setpoint_pub;

        setpoint_pub.pose.position.x = state->uavpose[0] + state->pid_feedback[0]; //In our axis, facing right
        setpoint_pub.pose.position.y = state->uavpose[1] + state->pid_feedback[1]; //In our axis, facing forward
        setpoint_pub.pose.position.z = state->uavpose[2] + state->pid_feedback[2]; //In our axix, facing up

        setpoint_pub.pose.orientation.w = 1;
        setpoint_pub.pose.orientation.x = state->uavorient[0] + state->pid_feedback[3];
        setpoint_pub.pose.orientation.y = state->uavorient[1] + state->pid_feedback[4];
        setpoint_pub.pose.orientation.z = state->uavorient[2] + state->pid_feedback[5];
        printf("x = %f y = %f, z = %f, ox = %f, oy = %f, oz = %f \n", setpoint_pub.pose.position.x, setpoint_pub.pose.position.y, setpoint_pub.pose.position.z,setpoint_pub.pose.orientation.x,setpoint_pub.pose.orientation.y,setpoint_pub.pose.orientation.z);
        Set_Pub.publish(setpoint_pub);
    }

    void data_publish()
    {
        geometry_msgs::Pose data_pub;

        //Position Error
        data_pub.position.x = state->uavpose_er[0];
        data_pub.position.y = state->uavpose_er[1];
        data_pub.position.z = state->uavpose_er[2];

        //Position Origin
        data_pub.orientation.x = state->uavpose_mocap[0];
        data_pub.orientation.y = state->uavpose_mocap[1];
        data_pub.orientation.z = state->uavpose_mocap[2];

        Data_Pub.publish(data_pub);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    UAV_State state;
    Drillpress_Setpoint dp_setpoint_ros;

    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    dp_setpoint_ros.init(&state);

    //the setpoint publishing rate MUST be faster than 20Hz
    ros::Rate rate(100.0);

    while (ros::ok())
    {

        dp_setpoint_ros.setpoint_publish();
        dp_setpoint_ros.data_publish();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}