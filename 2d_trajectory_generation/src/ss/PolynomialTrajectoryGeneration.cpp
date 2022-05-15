#include "gurobi_c++.h"
#include "PolynomialTrajectoryGeneration.h"
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
using Eigen::MatrixXd;
using std::ofstream;


void PolynomialTrajectory::Prodoftwoterms(int& ii, int& ll, int& rr, double& product){
/*
function to iterate the product of two terms
*/
    double prod = 1;
    for(int m = 0; m < rr; m++){
        prod *= (ii-m)*(ll-m);
    }
    product = prod;
    //return product;
}

void PolynomialTrajectory::Construct_Q(int& segments, int& N, int& r, VectorXd& T, MatrixXd& QQ)
{
/*
This function constructs Q matrix 
num_segments: number of segments
N: order of polynomial
r: derivative order
*/

    MatrixXd q = MatrixXd::Zero(N, N);
    MatrixXd _Q = MatrixXd::Zero(N*segments, N*segments);
    double prod1;
    for(int k = 0; k < segments; k++) 
    {
      for (int i = 0; i < N; i++) 
      {
        for (int l = 0; l < N; l++) 
          {
            if (i >= r and l >= r) 
            {
              PolynomialTrajectory::Prodoftwoterms(i, l, r, prod1);
              q(i, l) = 2 * prod1 * (pow(T(k+1), i + l - 2 * r + 1) - pow(T(k), i + l - 2 * r + 1)) / (i + l - 2 * r + 1);
                }
            }
        }

        _Q.block(k * 2 * r, k * N, 2 * r, N) = q;
    }
    QQ = _Q;
    //return QQ;
}

void PolynomialTrajectory::Construct_A(int& segments, int& N, int& r, VectorXd& T, MatrixXd& AA)
{
/*
This function constructs A matrix 
num_segments: number of segments
N: order of polynomial
r: derivative order
*/

    MatrixXd a = MatrixXd::Zero(2*r, N);
    MatrixXd _A = MatrixXd::Zero(2*r*segments, N*segments);
    std::vector<int> itlist;
    int product;
    for (int p = 0; p < segments; p++)
    {
      for (int i = 0; i < N; i++)
      {
        for (int j = 0; j < r; j++) 
          {
            if (i >= j) 
            {
              product = 1;
              for (int k = i - j + 1; k < i + 1; k++) 
                {
                  product *= k;
                }
              a(j, i) = product * pow(T(p), i - j);
              a(r + j, i) = product * pow(T(p+1), i - j);
            }
          }
      }
      _A.block(p * 2 * r, p * N, 2 * r, N) = a;
    }
    AA = _A;
    //return AA;
}




void PolynomialTrajectory::FindPolynomialCoeff(int& Nsegments, int& polynomial_order, int& r, vector<double>& start_point, vector<double>& end_point, 
                              double& av_speed, VectorXd& px, VectorXd& py, VectorXd& T, unsigned int& counter)
{
  try {
      int order = 2 * r * Nsegments;
      int N = polynomial_order + 1; // #of coeff are degree + 1
      vector<double> X(Nsegments+1); // sequence of X waypoints
      vector<double> Y(Nsegments+1); // sequence of Y waypoints


      X[0] = start_point[0];
      Y[0] = start_point[3];

      X[Nsegments] = end_point[0];
      Y[Nsegments] = end_point[1];



      for (int i=1; i < Nsegments; i++){
          X[i] = X[i-1] + (X[Nsegments] - X[0])/Nsegments;
          Y[i] = Y[i-1] + (Y[Nsegments] - Y[0])/Nsegments;
      }
    vector<double> bx = {start_point[0], start_point[1], start_point[2], end_point[0], 0.0, 0.0};
    vector<double> by = {start_point[3], start_point[4], start_point[5], end_point[1], 0.0, 0.0};
 
    vector<double> path_length(Nsegments);

    T(0) = 0;
    for (int i = 0; i < Nsegments; i++)
    {
       path_length[i] = sqrt((X[i+1] - X[i])*(X[i+1] - X[i]) + (Y[i+1] - Y[i])*(Y[i+1] - Y[i]));
       T(i+1) = T(i) + path_length[i]/av_speed;
    }
    /*
    vector<double> v_av(Nsegments+1);
    if (counter == 0)
    {
      v_av[0] = 0.0;
      v_av[1] = 0.5*av_speed;
      v_av[2] = av_speed;
      v_av[3] = av_speed;
      v_av[4] = 0.5*av_speed;
      v_av[5] = 0.0;

      for (int i = 1; i<= path_length.size(); i++)
      {
        T(i) = T(i-1) + 2*path_length[i-1]/(v_av[i]+v_av[i-1]);
      }
    }
    else if (counter == 1)
    {
      v_av[0] = 0.5*av_speed;
      v_av[1] = av_speed;
      v_av[2] = av_speed;
      v_av[3] = av_speed;
      v_av[4] = 0.5*av_speed;
      v_av[5] = 0.0;

      for (int i = 1; i<= path_length.size(); i++)
      {
        T(i) = T(i-1) + 2*path_length[i-1]/(v_av[i]+v_av[i-1]);
      }
    }
    else
    {
      v_av[0] = av_speed;
      v_av[1] = av_speed;
      v_av[2] = av_speed;
      v_av[3] = av_speed;
      v_av[4] = 0.5*av_speed;
      v_av[5] = 0.0;

      for (int i = 1; i<= path_length.size(); i++)
      {
        T(i) = T(i-1) + 2*path_length[i-1]/(v_av[i]+v_av[i-1]);
        
      }
    }
    */
    //cout << "T is:" << T(0) << "," << T(1) << ","<< T(2) << ","<< T(3) << ","<< T(4) << ","<< T(5) << endl;
    MatrixXd Q, A, A_inv, R;


    PolynomialTrajectory::Construct_Q(Nsegments, N, r, T, Q);

    PolynomialTrajectory::Construct_A(Nsegments, N, r, T, A);
    //cout << "Q" << Q <<"A"<< A << endl; 

    A_inv = A.inverse();//completeOrthogonalDecomposition().pseudoInverse();
    R = A_inv.transpose()*Q*A_inv;

    GRBEnv env = GRBEnv();

    GRBModel model = GRBModel(env);


    //Create variables
    double lb[N*Nsegments], ub[N*Nsegments], obj[N*Nsegments];
    char vtype[N*Nsegments];
    string decision_variables_x[N*Nsegments], decision_variables_y[N*Nsegments];
    for (int j = 0; j < N*Nsegments; j++){
        lb[j] = -GRB_INFINITY;
        ub[j] = GRB_INFINITY;
        obj[j] = 1;
        vtype[j] = GRB_CONTINUOUS;
        decision_variables_x[j] = "dx"+to_string(j);
        decision_variables_y[j] = "dy"+to_string(j);
    }


    GRBVar *dx = 0;
    GRBVar *dy = 0;
    dx = model.addVars(lb, ub, obj, vtype, decision_variables_x, order);
    dy = model.addVars(lb, ub, obj, vtype, decision_variables_y, order);


    // Set objective
    GRBQuadExpr obj_x = 0;
    for(int i = 0; i < order; i++)
    {
      GRBLinExpr _obj_x = 0;
      for (int j = 0; j < order; j++)
      {
        _obj_x += R(i, j) * dx[j];
      }
      obj_x +=  dx[i] * _obj_x; 
    } 
    //cout<< obj_x <<"\n";
    GRBQuadExpr obj_y = 0;
    for(int i = 0; i < order; i++)
    {
      GRBLinExpr _obj_y = 0;
      for (int j = 0; j < order; j++)
      {
        _obj_y += R(i, j) * dy[j];
      }
      obj_y +=  dy[i] * _obj_y; 
    } 
    
    model.setObjective(obj_x + obj_y, GRB_MINIMIZE);

    // Add constraints: 

    for (int k = 0; k < order; k++)
    {
      if (k < r)
      {
        model.addConstr(dx[k] == bx[k], "c0");
        model.addConstr(dy[k] == by[k], "c1");
      }
    }
   
    int j = 0;
    for( int k = 0; k < order; k++)
    {
      if (k >= order - r) 
      {
        model.addConstr(dx[k] == bx[r+j], "c2");
        model.addConstr(dy[k] == by[r+j], "c3");
        j += 1;
      } 
    }        


    int counter = 1;
    for (int n=r; n < order-2*r; n = n+2*r)
    {
      model.addConstr(dx[n] == X[counter], "c4");
      //model.addConstr(dx[n] <= X[counter] + 0.05, "c4");
      //model.addConstr(dx[n] >= X[counter] - 0.05, "c4");
      model.addConstr(dy[n] == Y[counter], "c5");
      //model.addConstr(dy[n] <= Y[counter] + 0.05, "c5");
      //model.addConstr(dy[n] >= Y[counter] - 0.05, "c5");
      counter += 1;
      for (int q = 0; q < r; q++)
      {
        model.addConstr(dx[n+q] == dx[n+q+r], "c6");
        model.addConstr(dy[n+q] == dy[n+q+r], "c7");
      }
    }


    // Optimize model
    //model.write("model.lp");
    model.set("OutputFlag", "0");
    //model.setParam("PSDtol", 1e6);
    //model.setParam("NumericFocus", 3);
    model.optimize();

    VectorXd Dx(order);
    VectorXd Dy(order);

 

    for (int i = 0; i < order; i++){
      Dx(i) = dx[i].get(GRB_DoubleAttr_X);
      Dy(i) = dy[i].get(GRB_DoubleAttr_X);
    }
    px = A_inv*Dx;
    py = A_inv*Dy;

  } 
  catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...) {
    cout << "Exception during optimization" << endl;
  }

}

void PolynomialTrajectory::FindPolynomialTraj(VectorXd& px, VectorXd& py, VectorXd& T, VectorXd& t, int& Nsegments, int& intervals, 
                               VectorXd& XX, VectorXd& YY, VectorXd& theta, VectorXd& XXdot, VectorXd& YYdot, VectorXd& thetadot, 
                               VectorXd& XXddot, VectorXd& YYddot)
{
    //VectorXd XXddot(intervals);
    //VectorXd YYddot(intervals);
    int k;
    for (int i = 0; i < t.size(); i++)
      {
        for (int j = 0; j < Nsegments+1; j++)
          {
             if( t(i) >= T[j] && t(i) < T[j+1])
               {
                 k = j;
               }
           } 
         XX(i) = px(6*k+0) + px(6*k+1)*t(i) + px(6*k+2)*pow(t(i),2) + px(6*k+3)*pow(t(i),3) + px(6*k+4)*pow(t(i),4) + px(6*k+5)*pow(t(i),5);
         YY(i) = py(6*k+0) + py(6*k+1)*t(i) + py(6*k+2)*pow(t(i),2) + py(6*k+3)*pow(t(i),3) + py(6*k+4)*pow(t(i),4) + py(6*k+5)*pow(t(i),5);

         XXdot(i) = px(6*k+1) + 2*px(6*k+2)*pow(t(i),1) + 3*px(6*k+3)*pow(t(i),2) + 4*px(6*k+4)*pow(t(i),3) + 5*px(6*k+5)*pow(t(i),4);
         YYdot(i) = py(6*k+1) + 2*py(6*k+2)*pow(t(i),1) + 3*py(6*k+3)*pow(t(i),2) + 4*py(6*k+4)*pow(t(i),3) + 5*py(6*k+5)*pow(t(i),4);

         XXddot(i) = 2*px(6*k+2) + 3*2*px(6*k+3)*pow(t(i),1) + 4*3*px(6*k+4)*pow(t(i),2) + 5*4*px(6*k+5)*pow(t(i),3);
         YYddot(i) = 2*py(6*k+2) + 3*2*py(6*k+3)*pow(t(i),1) + 4*3*py(6*k+4)*pow(t(i),2) + 5*4*py(6*k+5)*pow(t(i),3);

         //if (trajectory_counter == 0)
         //{
         //  theta(i) = 0.0;
         //  thetadot(i) = 0.0;
         //}
         //else
         //{
         //  theta(i) = atan2(YYdot(i),XXdot(i));
         //  thetadot(i) = (YYddot(i) * XXdot(i) - XXddot(i) * YYdot(i)) / (pow(XXdot(i), 2) + pow(YYdot(i), 2));
         //}
         

         theta(i) = atan2(YYdot(i),XXdot(i));
         thetadot(i) = (YYddot(i) * XXdot(i) - XXddot(i) * YYdot(i)) / (pow(XXdot(i), 2) + pow(YYdot(i), 2));


      }
    //trajectory_counter += 1;
    //cout << "trajectory_counter:" << trajectory_counter << endl;
}


void PolynomialTrajectory::getControlHorizonPoint(VectorXd &px,VectorXd &py, int& k, double& ch_time, vector<double>& ch_point)
{

    vector<double> point(8, 0.0);
    int kk = k-1;

    // position, velocity and acceleration in x direction
    point[0] = px(6*kk+0) + px(6*kk+1)*ch_time + px(6*kk+2)*pow(ch_time,2) + px(6*kk+3)*pow(ch_time,3) + px(6*kk+4)*pow(ch_time,4) + px(6*kk+5)*pow(ch_time,5);
    point[1] = px(6*kk+1) + 2*px(6*kk+2)*pow(ch_time,1) + 3*px(6*kk+3)*pow(ch_time,2) + 4*px(6*kk+4)*pow(ch_time,3) + 5*px(6*kk+5)*pow(ch_time,4);
    point[2] = 2*px(6*kk+2) + 3*2*px(6*kk+3)*pow(ch_time,1) + 4*3*px(6*kk+4)*pow(ch_time,2) + 5*4*px(6*kk+5)*pow(ch_time,3);

    // position, velocity and acceleration in y direction
    point[3] = py(6*kk+0) + py(6*kk+1)*ch_time + py(6*kk+2)*pow(ch_time,2) + py(6*kk+3)*pow(ch_time,3) + py(6*kk+4)*pow(ch_time,4) + py(6*kk+5)*pow(ch_time,5);
    point[4] = py(6*kk+1) + 2*py(6*kk+2)*pow(ch_time,1) + 3*py(6*kk+3)*pow(ch_time,2) + 4*py(6*kk+4)*pow(ch_time,3) + 5*py(6*kk+5)*pow(ch_time,4);
    point[5] = 2*py(6*kk+2) + 3*2*py(6*kk+3)*pow(ch_time,1) + 4*3*py(6*kk+4)*pow(ch_time,2) + 5*4*py(6*kk+5)*pow(ch_time,3);
    
    point[6] = atan2(point[4], point[1]);
    point[7] = (point[5] * point[1] - point[2] * point[4]) / (pow(point[1], 2) + pow(point[4], 2));
    ch_point = point;
}




