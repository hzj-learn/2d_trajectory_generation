#include "RayGeneration.h"
#include<ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <sstream>
#include <fstream>
#include <string> 
#include <vector>
#include <numeric>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf2/utils.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <TrajectoryGenerationOptimalControl.h>

using namespace std;
using namespace Eigen;

void RayGeneration:: GenerateEnsamble(vector<double>& current_pose, double& range_max, double& range_min, double& FOV, 
                                      double& resolution, vector< vector<double>>& points, vector<double>& ch_point)
{
  double current_yaw, x0, y0, d, r, x, y;
  current_yaw = current_pose[6];//tf2::getYaw(current_pose->pose.pose.orientation);
  x0 = current_pose[0];//->pose.pose.position.x;
  y0 = current_pose[3];//->pose.pose.position.y;

  int half_angle = 0.5 * FOV;
  int Nplanes = 5; // number of planes in which the points will be generated
  d = (range_max - range_min)/(Nplanes-1);
  //double r, x, y;
  vector<double> p(2,0);
  
  for (int i = 0; i < Nplanes; i++)
  { 
    r  = range_min + d * i;
    for (int j = -half_angle; j <= half_angle; j = j + 5)
    {
      x = r * cos(j*PI/180); 
      y = r* sin(j*PI/180); 

      p[0] = x0 + x * cos(current_yaw) - y * sin(current_yaw);
      p[1] = y0 + x * sin(current_yaw) + y * cos(current_yaw);
      points.push_back(p); 
    }
  }
 
}


void RayGeneration::CheckCollosion_GetCost(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, vector<double>& final_goal, double& av_speed, 
                                  vector<double>& start_point, vector< vector<double>>& points, vector< vector<double>>& availablepoints, 
                                  pcl::PointCloud<pcl::PointXYZ>& ensamble_cloud)
{
  float robot_radius = 0.4;
  float SR = 0.8;
  float factor = (1 + pow(SR, 4))/pow(SR, 4);
  bool EmptyPointCloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  if (cloud->points.size() != 0)
  {
    EmptyPointCloud = false;

    // this is not needed, i just need to make all z values to be zero, there has to be a smarter way
    for(int i = 0; i < cloud->points.size(); i++)
    {      
      pcl::PointXYZ pts;
      pts.x = cloud->points[i].x;
      pts.y = cloud->points[i].y;
      //pts.z = cloud_in->points[i].z;
      pts.z = 0.0;
      cloud_trans->points.push_back(pts);
    }
    kdtree.setInputCloud (cloud_trans);    
  }
  else
  {
    EmptyPointCloud = true;
  }

  int interval = 30;
  double T, t, r;
  VectorXd px(6), py(6);
  vector<double> point(3);
  int N = 1;
  OptimalControlTrajectory traj;

  if (EmptyPointCloud == false) // when the obstacles are there in the field of view
  {
    for (int i = 0; i < points.size(); i++)
    {
      traj.PolynomialCoeff(start_point, points[i], av_speed, px, py, T);    
      std::vector<float> distance_with_obstacles(interval);
      
      bool intersection_detected = false;
      for (int k = 0; k < interval; k++)
      {
   
        t = T*k/(interval-1);
        //std::vector<float> point(3);
        point[0] = px(0)*t*t*t*t*t/120 - px(1) * t*t*t*t/24 - px(2) * t*t*t/6 + px(3) * t*t/2 + px(4) * t + px(5);
        point[1] = py(0)*t*t*t*t*t/120 - py(1) * t*t*t*t/24 - py(2) * t*t*t/6 + py(3) * t*t/2 + py(4) * t + py(5);
        point[2]  = 0.0;
        pcl::PointXYZ pts_;
        pts_.x = point[0]; 
        pts_.y = point[1];
        pts_.z = point[2];

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if ( kdtree.radiusSearch (pts_, robot_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
          intersection_detected = true;
          break;
        }  

        std::vector<int> pointIdxNKNSearch(N);
        std::vector<float> pointNKNSquaredDistance(N);
        if ( kdtree.nearestKSearch (pts_, N, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        { 
          float dd = sqrt(pointNKNSquaredDistance[0]); // distance of this point to nearest obstacle
          float normalized_dd;
          if (dd - robot_radius < SR)
          {
            normalized_dd = factor* pow((pow(dd-robot_radius, 2)- SR*SR), 2) / (1 + pow((pow(dd-robot_radius, 2)- SR*SR), 2));
          }
          else{normalized_dd = 0;}          
          distance_with_obstacles[k] = normalized_dd;//sqrt(pointNKNSquaredDistance[0]); 
        }         
      }

      if (intersection_detected == false)
      {
        vector<double>a(4);
        // vector a contains x, y coordinate of the point, distance2goal cost and obstacle cost 
        a[0] = points[i][0];
        a[1] = points[i][1];
        a[2] = (final_goal[0]-points[i][0])*(final_goal[0]-points[i][0]) + (final_goal[1]-points[i][1])*(final_goal[1]-points[i][1]);
        a[3] = std::accumulate(distance_with_obstacles.begin(), distance_with_obstacles.end(), 0.0);
        //a[2] = sqrt((final_goal[0]-points[i][0])*(final_goal[0]-points[i][0]) + (final_goal[1]-points[i][1])*(final_goal[1]-points[i][1]));
      
        //double min_cost_of_traj = *std::min_element(distance_with_obstacles.begin(),distance_with_obstacles.end());
        //cout <<"distance with obstacle:" << min_cost_of_traj<<endl;
        //if (min_cost_of_traj - robot_radius < SR)
        //{
        //  a[3] = factor* pow((pow(min_cost_of_traj-robot_radius, 2)- SR*SR), 2) / (1 + pow((pow(min_cost_of_traj-robot_radius, 2)- SR*SR), 2));
        //}
        //else{a[3] = 0;}

        for (int k = 0; k < interval; k++)
        {
          pcl::PointXYZ _pts;
          double t = T*k/(interval-1);
          //std::vector<float> point(3);
          _pts.x = px(0)*t*t*t*t*t/120 - px(1) * t*t*t*t/24 - px(2) * t*t*t/6 + px(3) * t*t/2 + px(4) * t + px(5);
          _pts.y = py(0)*t*t*t*t*t/120 - py(1) * t*t*t*t/24 - py(2) * t*t*t/6 + py(3) * t*t/2 + py(4) * t + py(5);
          _pts.z  = 0.0;
          ensamble_cloud.points.push_back(_pts);
        }       
        
        availablepoints.push_back(a);

      }
    }
  }
  else // when there is no obstacles in the field of view, obtacle cost of all trajectories are zero 
  {
    for (int i = 0; i < points.size(); i++)
    {
      vector<double>a(4);
      a[0] = points[i][0];
      a[1] = points[i][1];
      a[2] = (final_goal[0]-points[i][0])*(final_goal[0]-points[i][0]) + (final_goal[1]-points[i][1])*(final_goal[1]-points[i][1]);         
      a[3] = 0.0;  
      availablepoints.push_back(a);
    }
  }
}

void RayGeneration::GenerateLocalGoal(vector< vector<double>>& availablepoints,  vector<double>& local_goal)
{

  double fd = 1;
  double fc = 10;

  vector<double> dist2finalgoal(availablepoints.size());
  vector<double> dist2goal_cost(availablepoints.size());
  vector<double> collision_cost(availablepoints.size());
  vector<double> total_cost(availablepoints.size());

  for (int i = 0; i < availablepoints.size(); i++)
  {
    dist2finalgoal[i] = availablepoints[i][2];
    collision_cost[i] = availablepoints[i][3];
  }

  vector<double> intermediate_point(2, 0);
  int IntermediatepointIndex = std::min_element(dist2finalgoal.begin(),dist2finalgoal.end()) - dist2finalgoal.begin();
  intermediate_point[0] = availablepoints[IntermediatepointIndex][0];
  intermediate_point[1] = availablepoints[IntermediatepointIndex][1];

  for (int i = 0; i < availablepoints.size(); i++)
  {
    dist2goal_cost[i] = sqrt(pow(intermediate_point[0]-availablepoints[i][0],2) + pow(intermediate_point[1]-availablepoints[i][1], 2));
  }

  double max_dist2goal_cost = *max_element(dist2goal_cost.begin(), dist2goal_cost.end());
  double max_collision_cost = *max_element(collision_cost.begin(), collision_cost.end());

  for (int i = 0; i < availablepoints.size(); i++)
  {

    total_cost[i] = fd * dist2goal_cost[i] / max_dist2goal_cost + fc * collision_cost[i];
    //cout << "i:"<<i<<" "<<"dist2goal:"<< dist2goal_cost[i]<<" "<<"collision:" <<collision_cost[i] <<" "<<"total"<< total_cost[i]<<endl;
  }

  int minElementIndex = std::min_element(total_cost.begin(),total_cost.end()) - total_cost.begin();
  local_goal[0] = availablepoints[minElementIndex][0];
  local_goal[1] = availablepoints[minElementIndex][1];
  //cout << "index of min cost" << minElementIndex<<endl;

}

