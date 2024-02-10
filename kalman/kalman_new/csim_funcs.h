#ifndef CSIM_FUNCS_H
#define CSIM_FUNCS_H


#include <cstdio>
//#include <eigen3/Eigen/src/Core/Matrix.h>
//#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <random>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <eigen3/Eigen/StdVector>

using std::cin; using std::cout;
using std::ofstream;
using std::ifstream;
using std::string;
using std::cerr;
using std::vector;
using std::endl;

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using Eigen::aligned_allocator;

class Filter {

    public:
    
    void mainLoop(int n);

    Filter(int dimensions[], vector<double> initial);
    

    int dm;

    int m;
    
    int u;

    vector<VectorXd, Eigen::aligned_allocator<VectorXd> > x;  //estimate in present

    vector<VectorXd, Eigen::aligned_allocator<VectorXd> > xest; //state estimates for next step

    vector<VectorXd, Eigen::aligned_allocator<VectorXd> > Y; //state measurements

    MatrixXd H; //sensor matrix, adjust once you have specifics

    //Kalman gain
    vector<MatrixXd, Eigen::aligned_allocator<MatrixXd> > K;

    MatrixXd A;  //state update

    MatrixXd B;  //Control input matrix

    vector<MatrixXd, Eigen::aligned_allocator<MatrixXd> > P;  //covariance matrix

    MatrixXd R;  //Adds noise to stuff (bit more than that really)
    //covariance matrix of measurement noise (general measurement error (diagonals) + which ones get noisy when other ones get noisy (non-diagonals))

    MatrixXd Q;         //analogue of R but for controls, so B.

    //Note: both are to be set initially and tweaked during testing. They do not change like P does.
    //can we assume they are both diagonal matrices?
    
    vector<VectorXd, Eigen::aligned_allocator<VectorXd> > U;  //represents controls

    MatrixXd I; //Identity matrix
    


    //vector<MatrixXd> q;

    vector<double> r; //list of variances for noise


    vector<MatrixXd, Eigen::aligned_allocator<MatrixXd> > PriorCov;

    vector<MatrixXd, Eigen::aligned_allocator<MatrixXd> > BasisChange;

};



#endif