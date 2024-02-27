#ifndef CSIM_FUNCS_H
#define CSIM_FUNCS_H


#include <cstdio>
//#include <eigen3/Eigen/src/Core/Matrix.h>
//#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <eigen3/Eigen/Dense>       //fix this. currently I'm importing way more than necessary
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

    //for all below, parameter n is the numbered iteration (timestep)
    void cycleXest(int n);  //gets state estimate for step n given info from step n-1

    void cyclePCov(int n);  //gets prior estimate for covariance matrix P

    void cyclePost(int n);  //updates P for step n+1 (posterior estimate)

    void cycleK(int n);     //updates Kalman gain for step n+1

    void cycleX(int n);     //gets current state estimate at step n
    
    void mainLoop(int n);   //cycles all of the above

    Filter(int dimensions[], vector<double> initial, MatrixXd a, MatrixXd b, MatrixXd h, MatrixXd p, MatrixXd q, MatrixXd r);
    //normally everything is set during the constructor, maybe add default and individual setters in the future 

    Filter(); //add matrix setters

    VectorXd getx(int n) {return x[n];}         //getter for x at time n

    VectorXd getxest(int n) {return xest[n];}   //getter for xest at time n


    void setY(int n, int i, double y) {Y[n][i] = y;}    //setter functions for inputs
    void setU(int n, int i, double u) {U[n][i] = u;}

    void setInputs(std::istringstream * ss1, std::istringstream * ss2, int n);  
    //takes the input sources and updates the objects Y and U accordingly


    
    private:

    int dm;     //amount of total dimensions in the state

    int m;      //amount of dimensions you can measure
    
    int u;      //amount of dimensions for controls

    vector<VectorXd, Eigen::aligned_allocator<VectorXd> > x;  //estimate in present

    vector<VectorXd, Eigen::aligned_allocator<VectorXd> > xest; //state estimates for next step

    vector<VectorXd, Eigen::aligned_allocator<VectorXd> > Y; //state measurements

    MatrixXd H; //sensor matrix, adjust once you have specifics
    
    vector<MatrixXd, Eigen::aligned_allocator<MatrixXd> > K; //Kalman gain

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

    vector<double> r; //list of variances for noise

    vector<MatrixXd, Eigen::aligned_allocator<MatrixXd> > PriorCov;     //matrix for prior covariance estimate

    vector<MatrixXd, Eigen::aligned_allocator<MatrixXd> > BasisChange;  //matrix for holding change of basis ahead of time

};



#endif