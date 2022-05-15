#include <iostream>
#include <ros/ros.h>
#include <tf2/utils.h>
#include "PolynomialTrajectoryGeneration.h"
//#include "RayGeneration.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <sstream>
#include <fstream>
#include <string> 
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <time.h>
#include<chrono>
#include<thread>

using namespace std;
using namespace Eigen;
//using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
//using std::chrono::system_clock;


ros::Publisher vel_pub;
ros::Publisher traj_pub;

class localplanner
{
    public: 
      localplanner() 
      {
        pose_msg_counter  = 0;
        ch_point = {0,0,0,0,0,0,0,0};
        curr_pose = {0,0,0,0,0,0,0,0};
        ContinueReplanning = true;
        FinalGoalinFOV = false;
      }; 
      void trajectory_callback(const nav_msgs::Odometry::ConstPtr& msg);
      void PublishTrajectory(VectorXd& XX, VectorXd& YY, VectorXd& theta, VectorXd& XXdot, 
                                     VectorXd& YYdot, VectorXd& thetadot, VectorXd& t);

    private: 
      int Nsegments = 5; // number of segments
      unsigned int pose_msg_counter;
      vector<double> ch_point;
      vector<double> curr_pose;     
      int polynomial_order = 5; // order of the polynomial
      int r = 3; // derivative order for min jerk trajectory
      double av_speed = 1.0; // a desired average speed
      double ch_time = 1.0; // control horizon time in seconds
      float delt = 0.02;
      int intervals;
      double Distance2Goal;
      double SensorRange = 3;
      bool ContinueReplanning;
      bool FinalGoalinFOV;
    
};

void localplanner::PublishTrajectory(VectorXd& XX, VectorXd& YY, VectorXd& theta, VectorXd& XXdot, 
                                     VectorXd& YYdot, VectorXd& thetadot, VectorXd& t)
{
    ros::Rate loop_rate(50);

    float start_time = ros::Time::now().toSec();

    for (int i = 0; i < t.size(); i++)
    {
      trajectory_msgs::MultiDOFJointTrajectory traj;
      //trajectory_msgs::MultiDOFJointTrajectoryPoint point;

      traj.header.frame_id = "base_link";
      traj.header.stamp = ros::Time::now();

      traj.points.resize(1);
      traj.joint_names.resize(1);
      traj.joint_names[0] ="jackal_base_link";

      trajectory_msgs::MultiDOFJointTrajectoryPoint point;
      point.transforms.resize(1);
      point.velocities.resize(1);
      point.accelerations.resize(1);

      float _time = start_time + t(i);

      point.transforms[0].translation.x = XX(i);
      point.transforms[0].translation.y = YY(i);

      tf2::Quaternion q;
      q.setRPY(0, 0, theta(i));
      point.transforms[0].rotation.x = q[0];
      point.transforms[0].rotation.y = q[1];
      point.transforms[0].rotation.z = q[2];
      point.transforms[0].rotation.w = q[3];

      point.velocities[0].linear.x = XXdot(i);
      point.velocities[0].linear.y = YYdot(i);
      point.velocities[0].angular.z = thetadot(i);

      point.time_from_start  = ros::Duration(_time);

      traj.points[0] = point;

      loop_rate.sleep();
      traj_pub.publish(traj);

    }
    ROS_INFO_STREAM("Publishing polynomial trajectory.");


}

void localplanner::trajectory_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{

    clock_t start, end;
    start = clock(); 

    vector<double> end_point(2, 0);
    //costmap_ros_ = std::make_shared<costmap_2d::Costmap2DROS>("costmap", *tf_);
    //RayGeneration::generateRays(msg);


    end_point[0] = 8.0;
    end_point[1] = -12.0;

    if (pose_msg_counter == 0)
    {
      // ch_point set to intial position of the robot
      ch_point[0] = msg->pose.pose.position.x;
      ch_point[3] = msg->pose.pose.position.y;      
      ch_point[1] = msg->twist.twist.linear.x;     
      ch_point[4] = msg->twist.twist.linear.y;
      ch_point[6] = tf2::getYaw(msg->pose.pose.orientation);
      ch_point[7] = msg->twist.twist.angular.z;
    }

    curr_pose[0] = msg->pose.pose.position.x;
    curr_pose[3] = msg->pose.pose.position.y;      
    curr_pose[1] = msg->twist.twist.linear.x;     
    curr_pose[4] = msg->twist.twist.linear.y;
    curr_pose[6] = tf2::getYaw(msg->pose.pose.orientation);
    curr_pose[7] = msg->twist.twist.angular.z;

 
    if (ContinueReplanning)
    {

      cout << "ch_point1:" << ch_time << "," << ch_point[0]<< ","<< ch_point[3]<< ","<<ch_point[6]<<endl;
      cout << "curr_pose:" << ch_time << "," << curr_pose[0]<< ","<< curr_pose[3]<< ","<<curr_pose[6]<<endl;

      VectorXd px(2 * r * Nsegments);
      VectorXd py(2 * r * Nsegments);
      VectorXd T(Nsegments+1);
  
      PolynomialTrajectory Traj;
      Traj.FindPolynomialCoeff(Nsegments, polynomial_order, r, ch_point, end_point, av_speed, px, py, T);
      cout << "T:" << T(0) << "," << T(1) << "," << T(2)<< ","<< T(3)<< ","<< T(4)<< ","<< T(5)<<endl;
      int k;
      if (!FinalGoalinFOV)
      {
        // k is the segment in which the ch_time falls
        for (int i = 0; i < T.size(); i++)
        {
          //cout << "i and T are:" << i << ", " << T[i] << endl;
          if (T[i] <= ch_time && T[i+1] > ch_time)
          {
            k = i+1;
          }
        }

        intervals = int(ch_time / delt)+1;

      }
      else
      { 
        k = Nsegments;
        ch_time = T(k);
        intervals = int(ch_time / delt)+1;
        ContinueReplanning = false;
        //cout << "replanning ends:" << k << "," << T.size() << "," << ch_time<< ","<< intervals<<endl;
      }

      VectorXd t(intervals);
      VectorXd XX(intervals);
      VectorXd YY(intervals);
      VectorXd theta(intervals);
      VectorXd XXdot(intervals);
      VectorXd YYdot(intervals);
      VectorXd XXddot(intervals);
      VectorXd YYddot(intervals);
      VectorXd thetadot(intervals);

      for (int i = 0; i < intervals; i++)
      { 
        if (i == 0){ t(i) = 0;}
        else{t(i) = t(i-1) + delt;}       
      }
      
      Traj.getControlHorizonPoint(px, py, k, ch_time, ch_point);
      
      Traj.FindPolynomialTraj(px, py, T, t, k, intervals, XX, YY, theta, XXdot, YYdot, thetadot, XXddot, YYddot);
      for (int i=0; i< t.size(); i++)
      {
      cout << "end point from traj:" << t(i) << "," << XX(i) << "," << YY(i)<< ","<< theta(i)<< endl;
      }
      cout << "ch_point2:" << ch_time << "," << ch_point[0]<< ","<< ch_point[3]<< ","<<ch_point[6]<<endl;
      PublishTrajectory(XX, YY, theta, XXdot, YYdot, thetadot, t);

      Distance2Goal = sqrt(pow(end_point[0]-ch_point[0], 2) +  pow(end_point[1]-ch_point[3], 2));
 
      if (Distance2Goal < SensorRange)
      { 
         FinalGoalinFOV = true;
      }
    }  
    else
    {
      cout << " The final goal is within the FOV, static trajectory is being tracked" << endl; 
    }


    end = clock(); 
    double time_taken2 = double(end - start) / double(CLOCKS_PER_SEC); 
    cout << "Time taken in publishing is : " << fixed << time_taken2 << setprecision(7) << "sec" << endl; 
    pose_msg_counter += 1;
    
    //double wait_for = ch_time - 0.05;
    //std::this_thread::sleep_for(std::chrono::seconds(wait_for));

}

int main(int argc, char **argv)
{


    // Launch our ros node
    ros::init(argc, argv, "trajectory_generation_node");
    ros::NodeHandle nh("~");

    //vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    bool latch;
    traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/jackal/desired_trajectory", 1, latch = true);

    std::string pose_topic;
    nh.param<std::string>("pose_topic", pose_topic, "/jackal/ground_truth/state");

    //std::string jointstate_topic;
    //nh.param<std::string>("jointstate_topic", jointstate_topic, "/joint_states");

    // Create subscribers
    localplanner lp;
    //os::Subscriber subodom1 = nh.subscribe(pose_topic.c_str(), 1, &localplanner::odometry_callback, &lp, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subodom2 = nh.subscribe(pose_topic.c_str(), 1, &localplanner::trajectory_callback, &lp, ros::TransportHints().tcpNoDelay());

    ROS_INFO("done...spinning to ros");
    ros::spin();

    return 0;
}

