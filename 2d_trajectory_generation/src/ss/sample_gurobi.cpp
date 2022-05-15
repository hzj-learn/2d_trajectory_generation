/* Copyright 2020, Gurobi Optimization, LLC */

/* This example formulates and solves the following simple QP model:

     minimize    x^2 + x*y + y^2 + y*z + z^2 + 2 x
     subject to  x + 2 y + 3 z >= 4
                 x +   y       >= 1
                 x, y, z non-negative

   It solves it once as a continuous model, and once as an integer model.
*/

#include "gurobi_c++.h"
#include <iostream>
#include <Eigen/Dense>
#include <list>
#include <stack>
#include <math.h>
#include <sstream>
#include <fstream>
#include <string> 


using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;
using std::ofstream;

double prodoftwoterms(int ii, int ll, int rr){
/*
function to iterate the product of two terms
*/
    double product = 1;
    for(int m = 0; m < rr; m++){
        //cout << "here"<< ii << " " << ll << " " <<m << "\n"<<endl;
        product *= (ii-m)*(ll-m);
    }
    return product;
}

MatrixXd construct_Q(int segments, int N, int r, VectorXd T)
{
/*
This function constructs Q matrix 
num_segments: number of segments
N: order of polynomial
r: derivative order
*/

    MatrixXd q = MatrixXd::Zero(N, N);
    MatrixXd QQ = MatrixXd::Zero(N*segments, N*segments);
    int prod;
    //cout << QQ << "\n ";
    for(int k = 0; k < segments; k++) 
    {
      for (int i = 0; i < N; i++) 
      {
        for (int l = 0; l < N; l++) 
          {
            if (i >= r and l >= r) 
            {
              prod = prodoftwoterms(i, l, r);
              //cout << "prod"<<" "<<prod<< " "<<i<<" "<<l<<" " <<r<<"\n"<<endl;
              q(i, l) = 2 * prod * (pow(T(k+1), i + l - 2 * r + 1) - pow(T(k), i + l - 2 * r + 1)) / (i + l - 2 * r + 1);
                }
            }
        }

        QQ.block(k * 2 * r, k * N, 2 * r, N) = q;
    }
    //cout << QQ << "\n ";
    return QQ;
}

MatrixXd construct_A(int segments, int N, int r, VectorXd T)
{
/*
This function constructs A matrix 
num_segments: number of segments
N: order of polynomial
r: derivative order
*/

    MatrixXd a = MatrixXd::Zero(2*r, N);
    MatrixXd AA = MatrixXd::Zero(2*r*segments, N*segments);
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
      AA.block(p * 2 * r, p * N, 2 * r, N) = a;
    }
    //cout << AA << "\n ";
    return AA;
}



void FindPolynomialCoeff(int Nsegments, int order, vector<double> &init_pos, vector<double> &init_vel, vector<double> &init_acc, vector<double> &end, double av_speed, int poly_order, int r, VectorXd &px, VectorXd &py, VectorXd &T)
{
  try {

      vector<double> X(Nsegments); // sequence of X waypoints
      vector<double> Y(Nsegments); // sequence of Y waypoints

      X[0] = init_pos[0];
      Y[0] = init_pos[1];
      X[Nsegments] = end[0];
      Y[Nsegments] = end[1];

      for (int i=1;i<Nsegments;i++){
          X[i] = X[i-1] + (X[Nsegments] - X[0])/Nsegments;
          Y[i] = Y[i-1] + (Y[Nsegments] - Y[0])/Nsegments;
      }
    vector<double> bx = {X[0], init_vel[0], init_acc[0], X[Nsegments], 0.0, 0.0};
    vector<double> by = {Y[0], init_vel[1], init_acc[1], Y[Nsegments], 0.0, 0.0};

    double path_length;

    T(0) = 0;
    for (int i = 1; i <= Nsegments; i++)
    {
       path_length = sqrt((X[i] - X[i-1])*(X[i] - X[i-1]) + (Y[i] - Y[i-1])*(Y[i] - Y[i-1]));
       T(i) = T(i-1) + path_length/av_speed;
    }


    Eigen::MatrixXd R;
    int N = poly_order + 1; // #of coeff are degree + 1
    MatrixXd Q, A, A_inv;

    Q = construct_Q(Nsegments, N, r, T);
    A = construct_A(Nsegments, N, r, T);
 

    A_inv = A.inverse();//completeOrthogonalDecomposition().pseudoInverse();
    R = A_inv.transpose()*Q*A_inv;
    //cout << "T" << T << "\n" << endl;
    //cout << "Q" << Q << "\n" << endl;   
    //cout << "A" << A_inv << "\n" << endl;

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
      model.addConstr(dy[n] == Y[counter], "c5");
      counter += 1;
      for (int q = 0; q < r; q++)
      {
        model.addConstr(dx[n+q] == dx[n+q+r], "c6");
        model.addConstr(dy[n+q] == dy[n+q+r], "c7");
      }
    }


    // Optimize model
    model.write("model.lp");
    //model.setParam("OutputFlag", 0);
    //model.setParam("PSDtol", 1e6);
    //model.setParam("NumericFocus", 3);
    model.optimize();

    VectorXd Dx(order);
    VectorXd Dy(order);

 

    for (int i = 0; i < order; i++){
    //cout << dx[i].get(GRB_StringAttr_VarName)[i] << " "
    //     << dx[i].get(GRB_DoubleAttr_X) << endl;
    //cout << dy[i].get(GRB_StringAttr_VarName) << " "
    //     << dy[i].get(GRB_DoubleAttr_X) << endl;
    Dx(i) = dx[i].get(GRB_DoubleAttr_X);
    Dy(i) = dy[i].get(GRB_DoubleAttr_X);

    }



    px = A_inv*Dx;
    py = A_inv*Dy;
   
    cout << "px: " << px << endl;
    cout << "py: " << py << endl;
    //cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;


  } catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...) {
    cout << "Exception during optimization" << endl;
  }

}



//-----------------------------------------------------------------------------------
// Not sure if we will keep this function

void FindPolynomialTraj(int Nsegments, int no_of_intervals, VectorXd &px, VectorXd &py, VectorXd &T, VectorXd &XX, VectorXd &YY, VectorXd &theta, VectorXd &t){


    VectorXd XXdot(Nsegments*no_of_intervals);
    VectorXd YYdot(Nsegments*no_of_intervals);
    int q = 0;
    for (int j=0;j<Nsegments;j++){

        for (int i = 0; i < no_of_intervals; i++){
            t(i+j*no_of_intervals) = T[j] + (T[j+1] - T[j]) * i / no_of_intervals;
            XX(i+j*no_of_intervals) = px(q) + px(q+1)*t(i+j*no_of_intervals) + px(q+2)*pow(t(i+j*no_of_intervals),2) + px(q+3)*pow(t(i+j*no_of_intervals),3)
                                      + px(q+4)*pow(t(i+j*no_of_intervals),4) + px(q+5)*pow(t(i+j*no_of_intervals),5);
            YY(i+j*no_of_intervals) = py(q) + py(q+1)*t(i+j*no_of_intervals) + py(q+2)*pow(t(i+j*no_of_intervals),2) + py(q+3)*pow(t(i+j*no_of_intervals),3)
                                      + py(q+4)*pow(t(i+j*no_of_intervals),4) + py(q+5)*pow(t(i+j*no_of_intervals),5);
            XXdot(i+j*no_of_intervals) = px(q+1) + 2*px(q+2)*pow(t(i+j*no_of_intervals),1) + 3*px(q+3)*pow(t(i+j*no_of_intervals),2)
                                         + 4*px(q+4)*pow(t(i+j*no_of_intervals),3) + 5*px(q+5)*pow(t(i+j*no_of_intervals),4);
            YYdot(i+j*no_of_intervals) = py(q+1) + 2*py(q+2)*pow(t(i+j*no_of_intervals),1) + 3*py(q+3)*pow(t(i+j*no_of_intervals),2)
                                         + 4*py(q+4)*pow(t(i+j*no_of_intervals),3) + 5*py(q+5)*pow(t(i+j*no_of_intervals),4);
            theta(i+j*no_of_intervals) = atan2(YYdot(i+j*no_of_intervals),XXdot(i+j*no_of_intervals));
        }
        q = q+6;

    }

}

//-----------------------------------------------------------------------------------


vector<double> PredictPoint(VectorXd &px,VectorXd &py, double replan_time){

    // returns the predicted pos, vel, acc of the replanning point

    vector<double> point_dyn = {0,0,0,0,0,0};

    point_dyn[0] = px(0) + px(1)*replan_time + px(2)*pow(replan_time,2) + px(3)*pow(replan_time,3) + px(4)*pow(replan_time,4) + px(5)*pow(replan_time,5);
    point_dyn[1] = py(0) + py(1)*replan_time + py(2)*pow(replan_time,2) + py(3)*pow(replan_time,3) + py(4)*pow(replan_time,4) + py(5)*pow(replan_time,5);

    point_dyn[2] = px(1) + 2*px(2)*pow(replan_time,1) + 3*px(3)*pow(replan_time,2) + 4*px(4)*pow(replan_time,3) + 5*px(5)*pow(replan_time,4);
    point_dyn[3] = py(1) + 2*py(2)*pow(replan_time,1) + 3*py(3)*pow(replan_time,2) + 4*py(4)*pow(replan_time,3) + 5*py(5)*pow(replan_time,4);

    point_dyn[4] = 2*px(2) + 3*2*px(3)*pow(replan_time,1) + 4*3*px(4)*pow(replan_time,2) + 5*4*px(5)*pow(replan_time,3);
    point_dyn[5] = 2*py(2) + 3*2*py(3)*pow(replan_time,1) + 4*3*py(4)*pow(replan_time,2) + 5*4*py(5)*pow(replan_time,3);


    return point_dyn;
}




int main(int argc, char *argv[])
{

    int Nsegments = 10; // number of segments
    vector<double> init_pos;
    vector<double> init_vel;
    vector<double> init_acc;
    vector<double> end;
    VectorXd T(Nsegments+1);
    double av_speed = 1.0; // a desired average speed
    int poly_order = 5; // order of the polynomial
    int r = 3; // derivative order for min jerk trajectory
    int no_of_intervals = 100;
    int order = 2 * r * Nsegments;
    VectorXd t(Nsegments*no_of_intervals);
    VectorXd XX(Nsegments*no_of_intervals);
    VectorXd YY(Nsegments*no_of_intervals);
    VectorXd theta(Nsegments*no_of_intervals);
    VectorXd px(order);
    VectorXd py(order);


    FindPolynomialCoeff(Nsegments, order, init_pos, init_vel, init_acc, end, av_speed, poly_order, r, px, py, T);
    FindPolynomialTraj(Nsegments, no_of_intervals, px,  py,  T,  XX,  YY,  theta,  t);

    ofstream outdata;

    // outdata.open("output _trajectory.txt");

    for (int i=0; i<Nsegments*no_of_intervals; i++){
        cout << t[i] << ", " << XX(i) << ", " << YY(i) << endl;
    }

    //outdata.close();

    return 0;
}