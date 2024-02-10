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

Filter::Filter(int dimensions[], vector<double> initial)
{

    dm = dimensions[0];
    m = dimensions[1];
    u = dimensions[2];

    //x[0] = new VectorXd(dm);  //estimate in present
    x.push_back(VectorXd(dm));
    xest.push_back(VectorXd(dm));
    for (int i=0; i<(int)initial.size(); i++)
    {
        x[0](i) = initial[i];
        // xest[0][i] = initial[i];
        // Y[0][i] = initial[i];
    }

    Eigen::Matrix3d a {{1,1,.5},
                    {0,1,1},
                    {0,0,1}};

    A = a; //set state update matrix (A) here

    Eigen::Vector3d b {.5,1,1};
    B = b; //set control input matrix (B) here

    Eigen::RowVector3d h = {0,0,1};
    H = h; //state -> measure matrix
    //H turns an x estimate format (dm x 1) to a y format (m x 1), so H is (m x dm)
    //takes a state prediction and turns it into what the sensors should get if that were the true state

    //cout << A << "\n\n";

// std::vector<Matrix<double,4,1>, Eigen::aligned_allocator<Matrix<double,4,1> > >
    K.push_back(MatrixXd::Zero(dm,m));

    Eigen::Matrix3d p {{1,0,0},
                    {0,1,0},
                    {0,0,.0025}};
    P.push_back(MatrixXd(3,3));  //covariance matrix (we dont set this initially?)
    P[0] = p;

    Q = MatrixXd::Identity(dm, dm)*.0025; //covariance matrix for B (quoting aaren)

    //cout << Q << endl;

    R = MatrixXd::Identity(m, m)*.0025;  //covariance matrix for noise of sensors

    I = MatrixXd::Identity(dm, dm);

    vector<MatrixXd> PriorCov;

    BasisChange.push_back(MatrixXd(dm,dm));


    //for above two matrices, notation BasisChange[n] means the BasisChange matrix USING data from step n, and the matrix is used in step n+1

}


//for first loop, n = 1, we need K[0] P[0] x[0] U[0] 

void Filter::mainLoop(int n)
{
    //for iteration n: predicts xest[n], then generates values for x[n], K[n], P[n] using n-1 data and xest[n]
    //calculates x[n] with xest[n], Previous K, and current Y


   
    xest.push_back(VectorXd(dm));
    //cout << A << B << x[n-1] << U[n-1];
    xest[n] = A*x[n-1] + B*U[n-1]; //        q? whats q??? apparently q is just the output from an N(0, Q) (vector?)
        //estimates for this state, n given n-1
        // possibly change to being Xest(n|n-1) and at the beginning of the loop instead (this change was made)

    
    PriorCov.push_back(MatrixXd(dm,dm));
    PriorCov[n-1] = (A*P[n-1]*A.transpose() + Q); //Prior covariance, using P(n-1)
        
    //new Kalman Gain for next step 
    // K(n+1)
    K.push_back(MatrixXd(dm,dm));
    MatrixXd K1 = PriorCov[n-1] * H.transpose();
    K[n] = K1 * (H*PriorCov[n-1]*H.transpose() + R).inverse();


    //updating Covariance Matrix for next step (posterior estimation)
    // P(n+1)
    BasisChange.push_back(MatrixXd(dm,dm));
    BasisChange[n] = I-K[n]*H; 
    P.push_back(MatrixXd(dm,dm));
    P[n] = BasisChange[n] * PriorCov[n-1] * BasisChange[n].transpose() + K[n]*R*(K[n]).transpose();


    x.push_back(VectorXd(dm));
    //cout << (Y[n] - H*xest[n]) << K[n-1] << endl;
    x[n] = xest[n] + K[n]*(Y[n] - H*xest[n]); //change Y[n-1] to Y[n]? (this change was made)
    
}


// product takes vertical of first and horizontal of second, 
// horizontal of first must equal vertical of second


//OLD FILE, DEPRECATED
//NEW ONE IN OTHER DIRECTORY

//void Filter::mainLoop(VectorXd Yn, VectorXd Un, int n)