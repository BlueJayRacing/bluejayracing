#include <cstdio>

//#include <Eigen/StdVector>

#include <iostream>
#include <fstream>
#include <random>
#include <string>
#include "csim_funcs.h"
#include <eigen3/Eigen/Dense>
#include <vector>

using std::cin; using std::cout;
using std::ofstream;
using std::ifstream;
using std::string;
using std::cerr;
using std::vector;
using std::endl;

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::RowVectorXd;



Filter::Filter(int dimensions[], vector<double> initial, MatrixXd a, MatrixXd b, MatrixXd h, MatrixXd p, MatrixXd q, MatrixXd r)
{

    dm = dimensions[0];
    m = dimensions[1];
    u = dimensions[2];

    (Y).push_back(VectorXd(m));
    (U).push_back(VectorXd(u));
    for (int j=0; j<m; j++) 
        Y[0](j) = 0;
    for (int j=0; j<u; j++) 
        U[0](j) = 0;

    x.push_back(VectorXd(dm));
    xest.push_back(VectorXd(dm));
    for (int i=0; i<(int)initial.size(); i++)
        x[0](i) = initial[i];

    A = a; //sets state update matrix (A) here

    B = b; //sets control input matrix (B) here

    H = h; //state -> measure matrix
    //H turns an x estimate format (dm x 1) to a y format (m x 1), so H is (m x dm)
    //takes a state prediction and turns it into what the sensors should get if that were the true state

// std::vector<Matrix<double,4,1>, Eigen::aligned_allocator<Matrix<double,4,1> > >
    K.push_back(MatrixXd::Zero(dm,m));
    P.push_back(MatrixXd(dm,dm));  //covariance matrix (we dont set this initially?)
    BasisChange.push_back(MatrixXd(dm,dm));

    P[0] = p;

    Q = q; //covariance matrix for B (quoting aaren)

    R = r;  //covariance matrix for noise of sensors

    I = MatrixXd::Identity(dm, dm);
    //for above two matrices, notation BasisChange[n] means the BasisChange matrix USING data from step n, and the matrix is used in step n+1

}


//for first loop, n = 1, we need K[0] P[0] x[0] U[0] 

void Filter::setInputs(std::istringstream * ss1, std::istringstream * ss2, int n)
{
    (Y).push_back(VectorXd(m));
    (U).push_back(VectorXd(u));
    string word;
    int i = 0;
    while(*ss1 >> word)          //measurements into Y[n]
        Y[n](i++) = stod(word);
    i = 0;
    while(*ss2 >> word)         //controls into U[n]
        U[n](i++) = stod(word);
}

void Filter::cycleXest(int n)
{
    xest.push_back(VectorXd(dm));
    //cout << A << B << x[n-1] << U[n-1];
    xest[n] = A*x[n-1] + B*U[n-1]; //        q? whats q??? apparently q is just the output from an N(0, Q) (vector?)
        //estimates for this state, n given n-1
        // possibly change to being Xest(n|n-1) and at the beginning of the loop instead (this change was made)

}

void Filter::cyclePCov(int n)
{
    PriorCov.push_back(MatrixXd(dm,dm));
    PriorCov[n-1] = (A*P[n-1]*A.transpose() + Q); //Prior covariance, using P(n-1)
}

void Filter::cycleK(int n)
{
    K.push_back(MatrixXd(dm,dm));
    MatrixXd K1 = PriorCov[n-1] * H.transpose();
    K[n] = K1 * (H*PriorCov[n-1]*H.transpose() + R).inverse();
}

void Filter::cyclePost(int n)
{
    BasisChange.push_back(MatrixXd(dm,dm));
    BasisChange[n] = I-K[n]*H; 
    P.push_back(MatrixXd(dm,dm));
    P[n] = BasisChange[n] * PriorCov[n-1] * BasisChange[n].transpose() + K[n]*R*(K[n]).transpose();
}

void Filter::cycleX(int n)
{
    x.push_back(VectorXd(dm));
    x[n] = xest[n] + K[n]*(Y[n] - H*xest[n]); //change Y[n-1] to Y[n]? (this change was made)
}

void Filter::mainLoop(int n)
{
    //part that sets up new inputs is in main

    //for iteration n: predicts xest[n], then generates values for x[n], K[n], P[n] using n-1 data and xest[n]
    //calculates x[n] with xest[n], Previous K, and current Y
    cycleXest(n); //update xest
    cyclePCov(n); //update PriorCov estimate
    cycleK(n);    //new Kalman Gain for next step n+1
    cyclePost(n); //updates covariance matrix(posterior estimation) P(n+1)
    cycleX(n);    //update x

}


// product takes vertical of first and horizontal of second, 
// horizontal of first must equal vertical of second