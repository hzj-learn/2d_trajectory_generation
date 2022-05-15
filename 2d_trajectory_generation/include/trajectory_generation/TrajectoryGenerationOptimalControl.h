#ifndef POLYNOMIAL_TRAJECTORY_H
#define POLYNOMIAL_TRAJECTORY_H
#include <iostream>
#include <Eigen/Dense>
#include <vector>

using std::vector;
using namespace Eigen;

class OptimalControlTrajectory
{ 
   public: 
     OptimalControlTrajectory() 
     {
       //trajectory_counter = 0;
     };

     void PolynomialCoeff(vector<double>& start_point, vector<double>& end_point, 
                              double& av_speed, VectorXd& px, VectorXd& py, double& T);

     void PolynomialTrajectory(VectorXd& px, VectorXd& py, double& T, VectorXd& t, VectorXd& XX, VectorXd& YY,
                               VectorXd& theta, VectorXd& XXdot, VectorXd& YYdot, VectorXd& thetadot, VectorXd& XXddot, VectorXd& YYddot);

     void getControlHorizonPoint(VectorXd& px,VectorXd& py, double& ch_time, vector<double>& ch_point);
   private:
     int trajectory_counter = 0;
   
};
#endif //POLYNOMIAL_TRAJECTORY_H
