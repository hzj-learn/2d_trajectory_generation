#include <iostream>
#include "PolynomialTrajectoryGeneration.h"
#include <Eigen/Dense>
#include <math.h>
#include <sstream>
#include <fstream>
#include <string> 
#include <vector>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    int Nsegments = 5; // number of segments
    // start_point is a point of the form (x, x_dot, x_ddot, y, y_dot, y_ddot}
    vector<double> start_point(6, 0.0);
    // of the form {x, y}
    vector<double> end_point(2, 0);
    end_point[0] = 3.0;
    end_point[1] = 3.0;

    
    double av_speed = 1.0; // a desired average speed
    int polynomial_order = 5; // order of the polynomial
    int r = 3; // derivative order for min jerk trajectory
    int intervals = 100;
    double ch_time = 2; // control horizon time in seconds

    //int order = 2 * r * Nsegments;
    VectorXd t(Nsegments * intervals);
    VectorXd XX(Nsegments * intervals);
    VectorXd YY(Nsegments * intervals);
    VectorXd theta(Nsegments * intervals);
    VectorXd XXdot(Nsegments * intervals);
    VectorXd YYdot(Nsegments * intervals);
    VectorXd thetadot(Nsegments * intervals);
    // declare control horizon point
    vector<double> ch_point(6, 0.0);
    
    VectorXd px(2 * r * Nsegments);
    VectorXd py(2 * r * Nsegments);
    VectorXd T(Nsegments+1);
    
    PolynomialTrajectory Traj;



    Traj.FindPolynomialCoeff(Nsegments, polynomial_order, r, start_point, end_point, av_speed, px, py, T);
    Traj.FindPolynomialTraj(px, py, T, Nsegments, intervals, XX, YY, theta, XXdot, YYdot, thetadot, t);
    Traj.getControlHorizonPoint(px, py, T, ch_time, ch_point);


    //ofstream outdata;
    cout << "control_horizon"<< ch_point[0] << ", " << ch_point[1] << ", " << ch_point[2] << ", " << ch_point[3] << ", " << ch_point[4] << ", " << ch_point[5] << endl;

    // outdata.open("output _trajectory.txt");

    for (int i=0; i<Nsegments*intervals; i++){
        cout << t[i] << ", " << XX(i) << ", " << YY(i) << endl;
    }

    //outdata.close();
    return 0;

}
