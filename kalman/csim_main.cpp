#include <cstdio>
//#include <eigen3/Eigen/src/Core/Matrix.h>
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
using std::vector;
using std::endl;

using Eigen::MatrixXd;
using Eigen::VectorXd;


int main(int argc, char** argv ) {
    argc++; //ignore


    //setting up inputs for measured estimates and controls, outputs for estimation in present and prediction of future state
    ifstream measures;
    ifstream controls;
    ofstream out;
    ofstream preout;
    measures.open(argv[1]);
    controls.open(argv[2]);
    out.open("est.txt"); //change to csv later on
    preout.open("predictions.txt");


    //set of inputs: 0-2: all positions 3-8: all velocities and accelerations, same pattern
    //9-10: pressure and temperature
    //11-18: angular positions and velocities of wheels


    //Set properties of the kalman filter below

    int dm = 3;     //dimension count, can change for later
    int m = 1;      //observed state counts
    int u = 1;      //control count
    int dim[] = {dm, m, u}; 

    

    //Set your initial state estimate HERE
    vector<double> initial = {0,0,1};

    //set A (dm x dm State update matrix)
    Eigen::MatrixXd a {{1,1,.5},
                    {0,1,1},
                    {0,0,1}};

    //set B (dm x u Control matrix)
    Eigen::MatrixXd b(dm, u);
    b << .5,1,1;

    //set H (m x dm Observation matrix)
    Eigen::MatrixXd h(m, dm); 
    h << 0,0,1;

    //set P (dm x dm state covariance matrix)
    Eigen::MatrixXd p {{1,0,0},
                    {0,1,0},
                    {0,0,.0025}};

    //set Q (adds noise)
    MatrixXd q = MatrixXd::Identity(dm, dm)*.0025;

    //set R (covariance matrix for sensor noise)
    MatrixXd r = MatrixXd::Identity(m, m)*.0025;


    Filter f_nep(dim, initial, a, b, h, p, q, r);




    std::default_random_engine rgen;
    std::normal_distribution<double> normal(0,1); //setting up random normal (if we need it for testing)

    int n = 1; //counts step number
    string line;
    string line2;
    out << "Format:              Position | Velocity | Acceleration\n";
    while (std::getline(measures, line) && std::getline(controls, line2)) { //continues as long as inputs go

        std::istringstream iss(line);
        std::istringstream iss2(line2);
        f_nep.setInputs(&iss, &iss2, n);    //loads input values into object

        f_nep.mainLoop(n);  //calls main loop, does calculations



        //below part is printing the values at each spot: completely optional and not entirely set up.
        out << "Estimates for time " << n << ":   ";
        for (int i=0; i<dm; i++)
            out << f_nep.getx(n)(i) << ", ";//write estimates to file here
        out << endl;
        preout << "Predictions for time " << n << ":   ";
        for (int i=0; i<dm; i++)
            preout << f_nep.getxest(n)(i) << ", ";//write predictions to file here
        preout << endl;

        n++;
    }
    return 0;
}

//Past code for reference (if anything goes wrong)


        //x[n] = xest[n] + K[n]*(Y[n] - H*xest[n]); //change Y[n] to Y[n-1]??


        // PriorCov[n] = (A*P[n]*A.transpose() + q); //Prior covariance, using P(n)
        // //new Kalman Gain for next step (whats R?)
        // // K(n+1)

        // for (int i=0; i<dm; i++)
        //     R[i] = normal(rgen)*r[i];

        // K[n+1] = PriorCov[n] * H.transpose() * (H*PriorCov[n]*H.transpose() + R).inverse();


        // //updating Covariance Matrix for next step (posterior estimation)
        // // P(n+1)
        // BasisChange[n] = I-K[n]*H;
        // P[n+1] = BasisChange[n] * PriorCov[n] * BasisChange[n].transpose() + K[n]*R[n]*(K[n]).transpose();

        
        // for (int i=0; i<18; i++)
        //     q[n][i]= normal(rgen)*Q[i,i]; //setting up q

        // xest[n+1] = A*x[n] + B*U[n] + q; //+ q         q? whats q??? apparently q is just the output from an N(0, Q)
        // //estimates for next state, n+1 given n
        // // Xest(n+1|n)




        //vector<MatrixXd> M(dm);

    // MatrixXd sample(dm, 1);

    // MatrixXd sample2(dm, 1);


    // vector<MatrixXd> x(dm);  //estimate in present

    // vector<MatrixXd> xest(dm, 1); //state estimates for next step

    // vector<MatrixXd> Y(m, 1); //state measurements

    // MatrixXd H(dm, dm); //sensor matrix, adjust once you have specifics

    // vector<MatrixXd> K(dm, dm); //Kalman gain

    // MatrixXd A(dm,dm);  //state update

    // MatrixXd B(dm,dm);  //Control input matrix

    // vector<MatrixXd> P(dm,dm);  //covariance matrix

    // vector<MatrixXd> R(dm,dm);  //Adds noise to stuff
    
    // vector<MatrixXd> U(dm, 1);  //represents controls

    // MatrixXd I(dm, dm); //Identity matrix

    // vector<MatrixXd> q(dm, 1);

    // vector<double> r; //list of variances for noise


    // vector<MatrixXd> PriorCov(dm, dm);

    // vector<MatrixXd> BasisChange(dm, dm);



    //  std::default_random_engine rgen;
    // std::normal_distribution<double> normal(0,1); //setting up random normal
    // VectorXd q(dm);
    // for (int i=0; i<dm; i++)
    //     q(i) = normal(rgen)*Q(i,i); //setting up q, might need to change later if assuming Q is non-diagonal (probably not actually)
    
    //cout << q << endl;