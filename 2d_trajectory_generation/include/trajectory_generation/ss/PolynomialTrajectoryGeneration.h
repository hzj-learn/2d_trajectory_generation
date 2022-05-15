#ifndef POLYNOMIAL_TRAJECTORY_H
#define POLYNOMIAL_TRAJECTORY_H
#include <iostream>
#include <Eigen/Dense>
#include <vector>

using std::vector;
using namespace Eigen;

class PolynomialTrajectory
{ 
   public: 
     PolynomialTrajectory() 
     {
       //trajectory_counter = 0;
     };
     void Prodoftwoterms(int& ii, int& ll, int& rr, double& product);

     void Construct_Q(int& segments, int& N, int& r, VectorXd& T, MatrixXd& QQ);

     void Construct_A(int& segments, int& N, int& r, VectorXd& T, MatrixXd& AA);

     void FindPolynomialCoeff(int& Nsegments, int& polynomial_order, int& r, vector<double>& start_point, vector<double>& end_point, 
                              double& av_speed, VectorXd& px, VectorXd& py, VectorXd& T, unsigned int& counter);


     void FindPolynomialTraj(VectorXd& px, VectorXd& py, VectorXd& T, VectorXd& t, int& Nsegments, int& intervals, 
                               VectorXd& XX, VectorXd& YY, VectorXd& theta, VectorXd& XXdot, VectorXd& YYdot, VectorXd& thetadot, VectorXd& XXddot, VectorXd& YYddot);

     void getControlHorizonPoint(VectorXd &px,VectorXd &py, int& k, double& ch_time, vector<double>& ch_point);
   private:
     int trajectory_counter = 0;
   
};
#endif //POLYNOMIAL_TRAJECTORY_H
