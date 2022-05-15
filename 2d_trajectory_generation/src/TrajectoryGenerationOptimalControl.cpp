#include "TrajectoryGenerationOptimalControl.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <list>
#include <stack>
#include <math.h>
#include <sstream>
#include <fstream>
#include <string> 
#include<vector>


using namespace std;
using namespace Eigen;
using std::ofstream;


void OptimalControlTrajectory::PolynomialCoeff(vector<double>& start_point, vector<double>& end_point, double& av_speed, VectorXd& px, VectorXd& py, double& T)
{
  double x0, vx0, ax0, xT, vxT, axT, y0, vy0, ay0, yT, vyT, ayT;
  x0 = start_point[0];
  vx0 = start_point[1];
  ax0 = start_point[2];
  xT = end_point[0];
  vxT = 0.0;
  axT = 0.0;

  y0 = start_point[3];
  vy0 = start_point[4];
  ay0 = start_point[5];
  yT = end_point[1];
  vyT = 0.0;
  ayT = 0.0;

  double path_length = sqrt(pow(xT-x0, 2) + pow(yT-y0, 2));
  T = path_length/av_speed;

  VectorXd delta_sx(3);
  VectorXd delta_sy(3);

  delta_sx(0) = xT - (x0 + vx0 * T + 0.5 * ax0 * T * T);
  delta_sx(1) = vxT - (vx0 + ax0 * T);
  delta_sx(2) = axT - ax0;

  delta_sy(0) = yT - (y0 + vy0 * T + 0.5 * ay0 * T * T);
  delta_sy(1) = vyT - (vy0 + ay0 * T);
  delta_sy(2) = ayT - ay0; 

  MatrixXd A(3,3);
  A(0,0) = 720/pow(T, 5);
  A(0,1) = -360/pow(T, 4);
  A(0,2) = 60/pow(T, 3);
  A(1,0) = 360/pow(T, 4);
  A(1,1) = -168/pow(T, 3);
  A(1,2) = 24/pow(T, 2);
  A(2,0) = -60/pow(T, 3);
  A(2,1) = 24/pow(T, 2);
  A(2,2) = -3/T;

  px(0) = A(0,0) * delta_sx(0) + A(0,1) * delta_sx(1) + A(0,2) * delta_sx(2);
  px(1) = A(1,0) * delta_sx(0) + A(1,1) * delta_sx(1) + A(1,2) * delta_sx(2);
  px(2) = A(2,0) * delta_sx(0) + A(2,1) * delta_sx(1) + A(2,2) * delta_sx(2);
  px(3) = ax0;
  px(4) = vx0;
  px(5) = x0;
  py(0) = A(0,0) * delta_sy(0) + A(0,1) * delta_sy(1) + A(0,2) * delta_sy(2);
  py(1) = A(1,0) * delta_sy(0) + A(1,1) * delta_sy(1) + A(1,2) * delta_sy(2);
  py(2) = A(2,0) * delta_sy(0) + A(2,1) * delta_sy(1) + A(2,2) * delta_sy(2);

  py(3) = ay0;
  py(4) = vy0;
  py(5) = y0;


}


void OptimalControlTrajectory::PolynomialTrajectory(VectorXd& px, VectorXd& py, double& T, VectorXd& t,  
                               VectorXd& XX, VectorXd& YY, VectorXd& theta, VectorXd& XXdot, VectorXd& YYdot, VectorXd& thetadot, 
                               VectorXd& XXddot, VectorXd& YYddot)
{

    for (int i = 0; i < t.size(); i++)
      {

        XX(i) = px(0)*pow(t(i), 5)/120 - px(1) * pow(t(i), 4)/24 - px(2) * pow(t(i), 3)/6 + px(3) * pow(t(i), 2)/2 + px(4) * t(i) + px(5);
        YY(i) = py(0)*pow(t(i), 5)/120 - py(1) * pow(t(i), 4)/24 - py(2) * pow(t(i), 3)/6 + py(3) * pow(t(i), 2)/2 + py(4) * t(i) + py(5);

        XXdot(i) = px(0)*pow(t(i), 4)/24 - px(1) * pow(t(i), 3)/6 - px(2) * pow(t(i), 2)/2 + px(3) * t(i) + px(4);
        YYdot(i) = py(0)*pow(t(i), 4)/24 - py(1) * pow(t(i), 3)/6 - py(2) * pow(t(i), 2)/2 + py(3) * t(i) + py(4);

        XXddot(i) = px(0)*pow(t(i), 3)/6 - px(1) * pow(t(i), 2)/2 - px(2) * t(i) + px(3);
        YYddot(i) = py(0)*pow(t(i), 3)/6 - py(1) * pow(t(i), 2)/2 - py(2) * t(i) + py(3);

        if(XXdot(i) != 0)
        {
          theta(i) = atan2(YYdot(i),XXdot(i));
        }
        else
        {
          theta(i) = theta(i-1);
        }
        
        if (XXdot(i) == 0 && YYdot(i)==0)
        {
          thetadot(i) = thetadot(i-1);
        }
        else
        {
          thetadot(i) = (YYddot(i) * XXdot(i) - XXddot(i) * YYdot(i)) / (pow(XXdot(i), 2) + pow(YYdot(i), 2));
        }     
        
      }

}


void OptimalControlTrajectory::getControlHorizonPoint(VectorXd &px,VectorXd &py, double& ch_time, vector<double>& ch_point)
{

  vector<double> point(8, 0.0);
  // position, velocity and acceleration in x direction
  point[0] = px(0)*pow(ch_time, 5)/120 - px(1) * pow(ch_time, 4)/24 - px(2) * pow(ch_time, 3)/6 + px(3) * pow(ch_time, 2)/2 + px(4) * ch_time + px(5);
  point[1] = px(0)*pow(ch_time, 4)/24 - px(1) * pow(ch_time, 3)/6 - px(2) * pow(ch_time, 2)/2 + px(3) * ch_time + px(4);
  point[2] = px(0)*pow(ch_time, 3)/6 - px(1) * pow(ch_time, 2)/2 - px(2) * ch_time + px(3);

  // position, velocity and acceleration in y direction
  point[3] = py(0)*pow(ch_time, 5)/120 - py(1) * pow(ch_time, 4)/24 - py(2) * pow(ch_time, 3)/6 + py(3) * pow(ch_time, 2)/2 + py(4) * ch_time + py(5);
  point[4] = py(0)*pow(ch_time, 4)/24 - py(1) * pow(ch_time, 3)/6 - py(2) * pow(ch_time, 2)/2 + py(3) * ch_time + py(4);
  point[5] = py(0)*pow(ch_time, 3)/6 - py(1) * pow(ch_time, 2)/2 - py(2) * ch_time + py(3);
    
  point[6] = atan2(point[4], point[1]);
  point[7] = (point[5] * point[1] - point[2] * point[4]) / (pow(point[1], 2) + pow(point[4], 2));
  ch_point = point;
}




