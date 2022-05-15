#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <string> 
#include <fstream>
#include <sstream>
#include <tf2/utils.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

ros::Publisher cmd_pub; 
using namespace std;
using namespace Eigen;
using std::ofstream;
std::ofstream outdata;

class controller
{
    public: 
      controller()
      {
        curr_pose = {0,0,0,0,0,0,0,0};
        sum_ex = 0.0;
        sum_ey = 0;
      };
      void cmd_callback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);
      void odometry_callback(const nav_msgs::Odometry::ConstPtr& _msg);
      //void PublishCommands(double &x, double &y, double &th, double &xdot, double &ydot, double &thdot, double &tt);    
    private: 
        std::vector <double> curr_pose;
        float epsilon = 0.075; 
        float k = 0.75;
        float sum_ex;
        float sum_ey;

};

void controller::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    // store odometry values in curr_pose array
    curr_pose[0] = msg->pose.pose.position.x;
    curr_pose[3] = msg->pose.pose.position.y;      
    curr_pose[1] = msg->twist.twist.linear.x;     
    curr_pose[4] = msg->twist.twist.linear.y; 
    curr_pose[6] = tf2::getYaw(msg->pose.pose.orientation);
    curr_pose[7] = msg->twist.twist.angular.z;

}

void controller::cmd_callback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) 
{

    geometry_msgs::Twist cmd_vel;
    float xdes, ydes, thetades, vxdes, vydes, wdes;
    float Zx_current, Zy_current, Zx_ref, Zy_ref, Zxdot_ref, Zydot_ref;
    float u1, u2, v, vx, vy, w;


    xdes = msg->points[0].transforms[0].translation.x;
    ydes = msg->points[0].transforms[0].translation.y;
    thetades = tf2::getYaw(msg->points[0].transforms[0].rotation);

    vxdes = msg->points[0].velocities[0].linear.x;
    vydes = msg->points[0].velocities[0].linear.y;
    wdes = msg->points[0].velocities[0].angular.z;

    Zx_current = curr_pose[0] + epsilon * cos(curr_pose[6]);
    Zy_current = curr_pose[3] + epsilon * sin(curr_pose[6]);

    float Zxdot_current, Zydot_current;
    Zxdot_current = curr_pose[1] - epsilon * curr_pose[7] * sin(curr_pose[6]);
    Zxdot_current = curr_pose[4] + epsilon * curr_pose[7] * cos(curr_pose[6]);

    Zx_ref = xdes + epsilon * cos(thetades);
    Zy_ref = ydes + epsilon * sin(thetades);

    Zxdot_ref = vxdes - epsilon * wdes * sin(thetades);
    Zydot_ref = vydes + epsilon * wdes * cos(thetades);

    double k2 = 0.05;

    sum_ex = sum_ex + (Zx_current - Zx_ref);
    sum_ey = sum_ey + (Zy_current - Zy_ref);

    u1 = Zxdot_ref - k * (Zx_current - Zx_ref);// - k2 * (Zxdot_current - Zxdot_ref);
    u2 = Zydot_ref - k * (Zy_current - Zy_ref);// - k2 * (Zydot_current - Zydot_ref);

    v = u1 * cos(curr_pose[6]) + u2 * sin(curr_pose[6]);
    vx = v * cos(curr_pose[6]);
    vy = v * sin(curr_pose[6]);

    w = (-u1 * sin(curr_pose[6]) + u2 * cos(curr_pose[6]))/epsilon;

    cmd_vel.linear.x = vx; 
    cmd_vel.linear.y = vy; 
    cmd_vel.angular.z = w;

    cmd_pub.publish(cmd_vel);

    outdata << curr_pose[0] << ", " << curr_pose[3] << ", " << curr_pose[6] << ", " << xdes << ", " << ydes << ", " << thetades << endl;

}

int main(int argc, char **argv)
{

    // Launch controller node
    ros::init(argc, argv, "trajectory_controller_node");
    ros::NodeHandle nh("~");

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

    std::string pose_topic;
    nh.param<std::string>("pose_topic", pose_topic, "/jackal/ground_truth/state");

    std::string trajectory_topic;
    nh.param<std::string>("trajectory_topic", trajectory_topic, "/jackal/desired_trajectory");

    // Create subscribers
    controller c;
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );

    char buffer [80];
    strftime (buffer,80,"gain - %d-%m-%Y %I:%M:%S.",now);
    outdata.open(buffer);
    ros::Subscriber sub1 = nh.subscribe(pose_topic.c_str(), 100, &controller::odometry_callback, &c, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub2 = nh.subscribe(trajectory_topic.c_str(), 100, &controller::cmd_callback, &c, ros::TransportHints().tcpNoDelay());

    ROS_INFO("done...spinning to ros");
    ros::spin();

    return 0;
}