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
#include<chrono>
#include<thread>

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
        global_counter = 0;
        Xi_previous = 0.0;
      };
      void cmd_callback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);
      void odometry_callback(const nav_msgs::Odometry::ConstPtr& _msg);
      //void PublishCommands(double &x, double &y, double &th, double &xdot, double &ydot, double &thdot, double &tt);    
    private: 
        std::vector <double> curr_pose;
        float epsilon = 0.3; 
        float k = 1.5;
        std::vector <double> xdes, ydes, vxdes, vydes, thetades, wdes, traj_global_time;
        std::vector <double> x1des, x2des, x3des;
        double Xi_previous;
        //std::vector<const> traj_global_time;
        int global_counter;
        bool TrajectoryPublished = false;

};



void controller::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    geometry_msgs::Twist cmd_vel;
    // store odometry values in curr_pose array
    double current_time = msg->header.stamp.toSec();

    curr_pose[0] = msg->pose.pose.position.x;
    curr_pose[3] = msg->pose.pose.position.y;      
    curr_pose[1] = msg->twist.twist.linear.x;     
    curr_pose[4] = msg->twist.twist.linear.y; 
    curr_pose[6] = tf2::getYaw(msg->pose.pose.orientation);
    curr_pose[7] = msg->twist.twist.angular.z;

    if (TrajectoryPublished == true)
    {
      auto it = std::lower_bound(traj_global_time.begin(), traj_global_time.end(), current_time);
      int j = std::distance(traj_global_time.begin(), it);
      //cout << "time index" << j<< "time:" << traj_global_time[j]<< "current_time:" << current_time<<endl;

      double delt = 0.02;      
      double u1 = 1/delt * (x1des[j] - curr_pose[0]);
      MatrixXd Ad(2,2);
      Ad(0,0) = 1.0;
      Ad(0,1) = 1.0;
      Ad(1,0) = delt * u1;
      Ad(1,1)  = 1.0;
      MatrixXd Bd(2,2);
      Bd(0,0) = delt/2;
      Bd(0,1) = delt/2;
      Bd(1,0) = 3 * pow(delt, 2) * u1/8;
      Bd(1,1)  = pow(delt, 2) * u1/8;    

      VectorXd Xi_f(2);
      Xi_f(0) = x2des[j];
      Xi_f(1) = x3des[j];
      VectorXd Xi_0(2);
      Xi_0(0) = tan(curr_pose[6]);
      Xi_0(1) = curr_pose[3];
      //cout << "Xi_0[0]:" << Xi_0[0] << "Xi_f[0]:" << Xi_f[0] << "Xi_0[1]:" << Xi_0[1] << "Xi_f[1]:" << Xi_f[1] << endl;
      VectorXd u2(2);
      u2 = Bd.inverse() * (Xi_f - Ad * Xi_0);

      VectorXd Xi = Ad * Xi_0 + Bd * u2;
      double w = (atan(Xi(0))- Xi_previous)/delt;      
      Xi_previous = atan(Xi(0));
      



      double v = u1/cos(curr_pose[6]);
      cmd_vel.linear.x = v ; 

      cout << "v:" << v << "w:" << w <<endl;
      //double w1 = u2[0] * pow(cos(curr_pose[6]), 2);
      //cmd_vel.angular.z =  w1;
      //cmd_pub.publish(cmd_vel);
      //std::this_thread::sleep_for(std::chrono::milliseconds(10));
      //double w2 = u2[1] * pow(cos(curr_pose[6]), 2);
      //cmd_vel.angular.z =  w2;
      //cmd_pub.publish(cmd_vel);      


      //cmd_vel.linear.x = v; 
      //cmd_vel.linear.y = u1 * tan(curr_pose[6]); 
      cmd_vel.angular.z = w;
      cmd_pub.publish(cmd_vel);

      outdata << curr_pose[0] << ", " << curr_pose[3] << ", " << curr_pose[1] << "," << curr_pose[4] << ", " <<   curr_pose[6] << ", " << xdes[j]
              << ", " << ydes[j] << ", " << vxdes[j] << ","<< vydes[j] << "," << thetades[j] << endl;
    }


}

void controller::cmd_callback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) 
{

    for (int i = 0; i < msg->points.size(); i++)
    {
      //int j = global_counter *  msg->points.size() + i;
      double tt = msg->points[i].time_from_start.toSec();
      //cout << "ttttt:" << tt<< endl;
      traj_global_time.push_back(tt);
      xdes.push_back(msg->points[i].transforms[0].translation.x);
      ydes.push_back(msg->points[i].transforms[0].translation.y);
      vxdes.push_back(msg->points[i].velocities[0].linear.x);
      vydes.push_back(msg->points[i].velocities[0].linear.y);
      double thdes = tf2::getYaw(msg->points[i].transforms[0].rotation);
      thetades.push_back(thdes);
      wdes.push_back(msg->points[i].velocities[0].angular.z);

      x1des.push_back(xdes.back());
      x2des.push_back(tan(thetades.back()));
      x3des.push_back(ydes.back());

      if (traj_global_time.size() >= 100)
      {
        traj_global_time.erase(traj_global_time.begin());
        xdes.erase(xdes.begin());
        ydes.erase(ydes.begin());
        vxdes.erase(vxdes.begin());
        vydes.erase(vydes.begin());
        thetades.erase(thetades.begin());
        wdes.erase(wdes.begin());
        x1des.erase(x1des.begin());
        x2des.erase(x2des.begin());
        x3des.erase(x3des.begin());
      }

    }
    
    TrajectoryPublished = true;

}

int main(int argc, char **argv)
{

    // Launch controller node
    ros::init(argc, argv, "trajectory_controller_node");
    ros::NodeHandle nh("~");
    bool latch;
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

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
    ros::Subscriber sub1 = nh.subscribe(pose_topic.c_str(), 1, &controller::odometry_callback, &c, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber sub2 = nh.subscribe(trajectory_topic.c_str(), 1, &controller::cmd_callback, &c, ros::TransportHints().tcpNoDelay(true));

    ROS_INFO("done...spinning to ros");
    ros::spin();

    return 0;
}