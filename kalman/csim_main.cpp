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
using std::cerr;
using std::vector;
using std::endl;

using Eigen::MatrixXd;
using Eigen::VectorXd;


int main(int argc, char** argv ) {

    argc++;
    ifstream measures;
    ifstream controls;
    ofstream out;
    ofstream preout;
    measures.open(argv[1]);
    controls.open(argv[2]);
    out.open("est.txt"); //change to csv later on
    preout.open("predictions.txt");

    //set of inputs: 0-2: all positions 3-8: all velocities and accelerations, same pattern
    //9-
    int dm = 3;  //dimension count, can change for later
    int m = 1;
    int u = 1;

    std::default_random_engine rgen;
    std::normal_distribution<double> normal(0,1); //setting up random normal

    int a[] = {dm, m, u}; //list of dimensions
    vector<double> initial = {0,0,1};
    
    Filter f_nep(a, initial);
    (f_nep.Y).push_back(VectorXd(m));
    (f_nep.U).push_back(VectorXd(u));
    for (int j=0; j<m; j++) 
        f_nep.Y[0](j) = 0;
    for (int j=0; j<u; j++) 
        f_nep.U[0](j) = 0;

    int n = 1;
    string line;
    string line2;

    out << "Format:              Position | Velocity | Acceleration\n";

    while (std::getline(measures, line) && std::getline(controls, line2)) {

        //cout << line << endl << line2 << endl;
        std::istringstream iss(line);
        std::istringstream iss2(line2);
        string word;
        int i=0;
        (f_nep.Y).push_back(VectorXd(m));
        (f_nep.U).push_back(VectorXd(u));

        while(iss >> word)          //measurements into Y[n]
        {
            //cout << "scanning in element " << word << endl;
            f_nep.Y[n](i++) = stod(word);
        }
        i = 0;
        while(iss2 >> word)         //controls into U[n]
            f_nep.U[n](i++) = stod(word);
        //cout << f_nep.Y[n] << "    " << f_nep.U[n] << endl;


        f_nep.mainLoop(n);


        //below part is printing the values at each spot: completely optional and not entirely set up.
        out << "Estimates for time " << n << ":   ";
        for (int i=0; i<dm; i++)
            out << f_nep.x[n](i) << ", ";//write estimates to file here
        out << endl;
        preout << "Predictions for time " << n << ":   ";
        for (int i=0; i<dm; i++)
            preout << f_nep.xest[n](i) << ", ";//write predictions to file here
        preout << endl;
        
        n++;
    }
    return 0;
}

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