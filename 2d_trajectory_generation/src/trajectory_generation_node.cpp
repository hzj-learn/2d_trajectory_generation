#include <iostream>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <TrajectoryGenerationOptimalControl.h>
#include <RayGeneration.h>
#include <Eigen/Dense>
#include <math.h>
#include <sstream>
#include <fstream>
#include <string> 
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include<chrono>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Geometry>
#include <pcl/filters/frustum_culling.h>


using namespace std;
using namespace Eigen;
std::ofstream outdata;
Eigen::Matrix4f lidar_pose;

ros::Publisher vel_pub, traj_pub, best_traj_pub, ensamble_pub;

class localplanner
{
    public: 
      localplanner() 
      {
        pose_msg_counter  = 0;
        ch_point = {0,0,0,0,0,0,0,0};
        curr_pose = {0,0,0,0,0,0,0,0};
        final_goal = {20.0, 1.0};
        ContinueReplanning = true;
        FinalGoalinFOV = false;
        //costmap_ros_ = NULL;
      }; 
      void trajectory_callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
      void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
      void PublishTrajectory(const sensor_msgs::PointCloud2ConstPtr& cloud, VectorXd& XX, VectorXd& YY, VectorXd& theta, VectorXd& XXdot, 
                                     VectorXd& YYdot, VectorXd& thetadot, VectorXd& t, VectorXd& XXddot, VectorXd& YYddot);
    private: 
      int Nsegments = 5; // number of segments
      unsigned int pose_msg_counter;
      vector<double> ch_point;
      vector<double> curr_pose; 
      vector<double> final_goal;   
      int polynomial_order = 5; // order of the polynomial
      int r = 3; // derivative order for min jerk trajectory
      double av_speed = 2.0; // desired average speed
      //nh.param<double>("average_speed", av_speed, 1.0);
      double ch_time = 0.11; // control horizon time in seconds
      float delt = 0.05;
      int intervals;
      double Distance2Goal;
      double SensorRangeMax = 5.0; // range of sensor in meters
      double SensorRangeMin = 2.0; // range of sensor in meters
      double SensorFOV = 120.0; // field of view for planning in degrees
      double GridResolution = 0.25; // resolution of point ensamble grid in meters
      bool ContinueReplanning;
      bool FinalGoalinFOV;
      pcl::PointCloud<pcl::PointXYZ> traj_cloud;
    
};


void localplanner::PublishTrajectory(const sensor_msgs::PointCloud2ConstPtr& cloud, VectorXd& XX, VectorXd& YY, VectorXd& theta, VectorXd& XXdot, 
                                     VectorXd& YYdot, VectorXd& thetadot, VectorXd& t, VectorXd& XXddot, VectorXd& YYddot)
{
    //ros::Rate loop_rate(50);

    
    trajectory_msgs::MultiDOFJointTrajectory traj;
    traj.points.resize(t.size());
    traj.joint_names.resize(1);
    traj.joint_names[0] ="jackal_base_link";
    traj.header.frame_id = "map";
    traj.header.stamp = ros::Time::now();//cloud->header.stamp;
    double start_time = ros::Time::now().toSec();

    for (int i = 0; i < t.size(); i++)
    {
      outdata << t(i) << "," << XX(i) << "," << YY(i) << "," << XXdot(i) << "," << YYdot(i) << "," << theta(i) << "," << endl;
      trajectory_msgs::MultiDOFJointTrajectoryPoint point;
      point.transforms.resize(1);
      point.velocities.resize(1);
      point.accelerations.resize(1);

      double _time = start_time + t(i);

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

      point.accelerations[0].linear.x = XXddot(i);
      point.accelerations[0].linear.y = YYddot(i);

      point.time_from_start  = ros::Duration(_time);
      traj.points[i] = point;      
      
      //std::this_thread::sleep_for(std::chrono::milliseconds(20));
      //loop_rate.sleep();
    }
    traj_pub.publish(traj);
    ROS_INFO_STREAM("Publishing polynomial trajectory.");


}


void localplanner::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (pose_msg_counter == 0)
  {
    // ch_point set to intial position of the robot
    ch_point[0] = msg->pose.pose.position.x;
    ch_point[1] = 0.1;//msg->twist.twist.linear.x; //started with a small initial velocity always
    ch_point[2] = 0.0; // sensor doesnt give acceleration, so a small initial value
    ch_point[3] = msg->pose.pose.position.y;         
    ch_point[4] = msg->twist.twist.linear.y; // no need to give any velocity in y direction 
    ch_point[5] = 0.0; //initilized with 0 accelration 
    ch_point[6] = tf2::getYaw(msg->pose.pose.orientation);
    ch_point[7] = msg->twist.twist.angular.z;
  }

  curr_pose[0] = msg->pose.pose.position.x;
  curr_pose[1] = msg->twist.twist.linear.x;
  curr_pose[2] = 0.0;
  curr_pose[3] = msg->pose.pose.position.y;      
  curr_pose[4] = msg->twist.twist.linear.y;
  curr_pose[5] = 0.0;
  curr_pose[6] = tf2::getYaw(msg->pose.pose.orientation);
  curr_pose[7] = msg->twist.twist.angular.z;


  Eigen::Quaternionf q;
  q.w() = msg->pose.pose.orientation.w;
  q.x() = msg->pose.pose.orientation.x;
  q.y() = msg->pose.pose.orientation.y;
  q.z() = msg->pose.pose.orientation.z;
  Eigen::Matrix3f mat = q.toRotationMatrix();
  lidar_pose = Eigen::Matrix4f::Identity();
  lidar_pose.block(0,0,3,3) = mat;
  lidar_pose(0,3) = msg->pose.pose.position.x;
  lidar_pose(1,3) = msg->pose.pose.position.y;
  lidar_pose(2,3) = msg->pose.pose.position.z;

  pose_msg_counter += 1;
}

void localplanner::trajectory_callback(const sensor_msgs::PointCloud2ConstPtr& cloud) 
{

    clock_t start, end;
    start = clock(); 

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZ>);

    
    //pcl::fromROSMsg(cloud_out, *cloud_in);
    pcl::fromROSMsg(*cloud, *cloud_in);

    pcl::transformPointCloud (*cloud_in, *cloud_trans, lidar_pose);

    pcl::FrustumCulling<pcl::PointXYZ> fc;
    fc.setInputCloud (cloud_trans);
    fc.setVerticalFOV (SensorFOV);
    fc.setHorizontalFOV (SensorFOV);
    fc.setNearPlaneDistance (0.0);
    fc.setFarPlaneDistance (SensorRangeMax);

    // .. read or input the camera pose from a registration algorithm.
    fc.setCameraPose (lidar_pose);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr truncated_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    fc.filter (*truncated_cloud);

    
    RayGeneration rg;
    vector< vector<double> > points;
    vector< vector<double> > availablepoints;
    //vector<double> proximity2goal;
    vector<double> local_goal(2);

    pcl::PointCloud<pcl::PointXYZ> ensamble_cloud;

    rg.GenerateEnsamble(curr_pose, SensorRangeMax, SensorRangeMin, SensorFOV, GridResolution, points, ch_point);
    rg.CheckCollosion_GetCost(truncated_cloud, final_goal, av_speed, ch_point, points, availablepoints, ensamble_cloud);    
    rg.GenerateLocalGoal(availablepoints, local_goal);
    cout <<"local goal:"<< local_goal[0]<<" "<< local_goal[1]<<endl;
    cout <<"current position:"<< curr_pose[0]<<" "<< curr_pose[3]<<endl;
    cout << "ContinueReplanning:"<<" "<<ContinueReplanning<<" "<<"FinalGoalinFOV"<<" "<<FinalGoalinFOV<<endl;


    if (ContinueReplanning)
    {
      VectorXd px(6);
      VectorXd py(6);
      double T;
  
      OptimalControlTrajectory Traj;
      Traj.PolynomialCoeff(ch_point, local_goal, av_speed, px, py, T);

      
      Distance2Goal = sqrt(pow(final_goal[0]-curr_pose[0], 2) +  pow(final_goal[1]-curr_pose[3], 2));
 
      if (Distance2Goal < SensorRangeMax){ FinalGoalinFOV = true;}

      if (!FinalGoalinFOV)
      {
        intervals = int(ch_time / delt)+1;
      }
      else
      { 
        ch_time = T;
        intervals = int(ch_time / delt)+1;
        ContinueReplanning = false;
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
      
      Traj.getControlHorizonPoint(px, py, ch_time, ch_point);
      //cout << "ch_point2:" << ch_time << "," << ch_point[0]<< ","<< ch_point[3]<< ","<<ch_point[6]<<endl;
      Traj.PolynomialTrajectory(px, py, T, t, XX, YY, theta, XXdot, YYdot, thetadot, XXddot, YYddot);

      //pcl::PointCloud<pcl::PointXYZ> traj_cloud;
      
      for(int i = 0; i < XX.size(); i++)
      {
        pcl::PointXYZ pt;
        pt.x = XX(i);
        pt.y = YY(i);
        pt.z = 0.0;
        traj_cloud.points.push_back(pt);
      }
      //sensor_msgs::PointCloud2 pc_traj;
      //pcl::toROSMsg(traj_cloud, pc_traj);
      //pc_traj.header.frame_id = "velodyne";
      //pc_traj.header.stamp = ros::Time::now();
      //best_traj_pub.publish(pc_traj);

    
      PublishTrajectory(cloud, XX, YY, theta, XXdot, YYdot, thetadot, t, XXddot, YYddot);

    }  
    else
    {
      cout << " The final goal is within the FOV, static trajectory is being tracked" << endl; 
    }
    //cout << "total points in trajectory is:"<<traj_cloud.size()<<endl;
    sensor_msgs::PointCloud2 pc_traj;
    pcl::toROSMsg(traj_cloud, pc_traj);
    pc_traj.header.frame_id = "map";
    pc_traj.header.stamp = ros::Time::now();
    best_traj_pub.publish(pc_traj);


    sensor_msgs::PointCloud2 pc_ensamble;
    pcl::toROSMsg(ensamble_cloud, pc_ensamble);
    pc_ensamble.header.frame_id = "map";
    pc_ensamble.header.stamp = ros::Time::now();
    ensamble_pub.publish(pc_ensamble);

    end = clock(); 
    double time_taken2 = double(end - start) / double(CLOCKS_PER_SEC); 
    //std::chrono::milliseconds tt = time_taken2 *1000;
    cout << "Time taken in publishing is : " << fixed << time_taken2 << setprecision(7) << "sec" << endl; 
    //pose_msg_counter += 1;
    
    //double wait_for = ch_time - 0.05;
    //std::this_thread::sleep_for(std::chrono::seconds(wait_for));
    //loop_rate.sleep();
    //std::this_thread::sleep_for(std::chrono::milliseconds(200-time_taken2));
    //sleep(0.2- time_taken2);
}

int main(int argc, char **argv)
{

    // Launch our ros node
    ros::init(argc, argv, "trajectory_generation_node");
    ros::NodeHandle nh("~");

    //vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    bool latch;
    traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/jackal/desired_trajectory", 1, latch=true);
    best_traj_pub = nh.advertise<sensor_msgs::PointCloud2>("/best_trajectory", 1, true);
    ensamble_pub = nh.advertise<sensor_msgs::PointCloud2>("/ensamble", 1, true);


    std::string pointcloud_topic;
    nh.param<std::string>("pointcloud_topic", pointcloud_topic, "/passthrough2/output");

    std::string odometry_topic;
    nh.param<std::string>("odometry_topic", odometry_topic, "/jackal/ground_truth/state");



    // Create subscribers
    localplanner lp;
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    char buffer [80];
    strftime (buffer,80,"trajectory - %d-%m-%Y %I:%M:%S.",now);
    outdata.open(buffer);
    ros::Subscriber subpc = nh.subscribe(pointcloud_topic.c_str(), 1, &localplanner::trajectory_callback, &lp, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subodom = nh.subscribe(odometry_topic.c_str(), 1, &localplanner::odometry_callback, &lp, ros::TransportHints().tcpNoDelay());

    ROS_INFO("done...spinning to ros");
    ros::spin();

    return 0;
}

