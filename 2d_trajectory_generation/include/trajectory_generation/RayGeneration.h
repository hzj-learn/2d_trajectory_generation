#ifndef REYGENERATION_H
#define REYGENERATION_H
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>


using namespace std;
using namespace Eigen;

class RayGeneration
{ 
  public: 
    RayGeneration() {};
    void GenerateEnsamble(vector<double>& current_pose, double &range_max, double &range_min, double &FOV, double &resolution,
                          vector< vector<double>> &points, vector<double> &ch_point);

    void CheckCollosion_GetCost(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, vector<double>& final_goal, double& av_speed, vector<double>& start_point, 
                        vector< vector<double>>& points, vector< vector<double>>& availablepoints, pcl::PointCloud<pcl::PointXYZ>& ensamble_cloud);

    void GenerateLocalGoal(vector< vector<double>>& availablepoints, vector<double>& local_goal);

  private: 
    double PI = 3.1416;



};
#endif //REYGENERATION_H
