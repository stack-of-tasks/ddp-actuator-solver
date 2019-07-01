#include <iostream>
#include <fstream>

#include <ddp-actuator-solver/ddpsolver.hh>
#include "PyreneActuator.hh"
#include "pyreneCostFunction.hh"

#include <time.h>
#include <sys/time.h>
#include <vector>
#include <string>

using namespace std;
using namespace Eigen;


vector<double> fillVector(string repoBags, string fileName){
    vector<double> fillVector(1000);
    ifstream file((repoBags + fileName).c_str(),ios::in);
    if (!file){
        cerr << "File " << fileName.c_str() << " not Read"<< endl;
    }
    
    double val;
    for (int j=0; j<1000; j++) {
        string line;
        getline(file, line);
        stringstream ss(line);
        if (ss >> val){
            fillVector[j] = val;
        }
    }
    return fillVector;
}

int main (int argc, char *argv[])
{
    if (argc != 2){ 
        cerr << "Convention: mainBags nameRepoBags " << endl;
        return 1;
    }
    string repoBags = argv[1];

    vector<double> vec_joint_pos = fillVector(repoBags, "joint_pos.txt");

    struct timeval tbegin,tend;
    double texec=0.0;
    DDPSolver<double,2,1>::stateVec_t xinit,xDes,x;
    DDPSolver<double,2,1>::commandVec_t u;

    unsigned int T = 50;
    double dt=1e-3;
    unsigned int iterMax = 100;
    double stopCrit = 1e-3; //0.01;
    DDPSolver<double,2,1>::stateVecTab_t xList;
    DDPSolver<double,2,1>::commandVecTab_t uList;
    DDPSolver<double,2,1>::traj lastTraj;

    PyreneActuator PyreneActuator;
    CostFunctionPyreneActuator costFunction;
    costFunction.setTauLimit(70);
    costFunction.setJointLimit(0.0, -2.35619449019);
    costFunction.setJointVelLimit(30.0, -30.0);
    DDPSolver<double,2,1> testSolverActuator(PyreneActuator,costFunction,DISABLE_FULLDDP,DISABLE_QPBOX); 

    double dx_joint;
    dx_joint = 0.5422;
    xinit << vec_joint_pos[0],
             dx_joint;
    
         
    for (int i=0; i<999; i++) {

        xDes << vec_joint_pos[i+1], 0.0; 
        
        testSolverActuator.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);

        int N = 100;
        gettimeofday(&tbegin,NULL);
        for(int i=0;i<N;i++) testSolverActuator.solveTrajectory();
        gettimeofday(&tend,NULL);

        lastTraj = testSolverActuator.getLastSolvedTrajectory();
        xList = lastTraj.xList;
        uList = lastTraj.uList;
        unsigned int iter = lastTraj.iter;

        xinit << xList[T](0,0),
                 xList[T](1,0);


        texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
        texec /= N;


        // cout << costFunction.running_cost << endl;
        // cout << costFunction.TauConstraints << endl;
        // cout << costFunction.dTauConstraints << endl;
        cout << endl;
        cout << "temps d'execution total du solveur ";
        cout << texec << endl;
        cout << "temps d'execution par pas de temps ";
        cout << texec/T << endl;
        cout << "Nombre d'itérations : " << iter << endl;

        string ResultNumber;
        ostringstream convert;
        convert << i;
        ResultNumber = convert.str();
             
        ofstream file(("results_sinu/results_sinu"+ ResultNumber + ".csv").c_str(),ios::out | ios::trunc);
        if(file)
        {   
            file << "q,qdes,qDot,qDotDes,u" << endl;
            for(int i=0;i<T;i++){
                file << xList[i](0,0) << "," <<  xDes[0] << "," << xList[i](1,0) << "," << xDes[1] << "," << uList[i-1](0,0) << endl;
            } 
            file << xList[T](0,0) << "," <<  xDes[0] << "," << xList[T](1,0) << "," << xDes[1] << "," << uList[T-1](0,0) << endl;
            file.close();
        }
        else{
            cerr << "erreur ouverte fichier" << endl;
        }

        if (i==0){
            ofstream fichier(("results_sinu/results_sinu.csv"),ios::out | ios::trunc);
            if(fichier){
                fichier << "q,qdes,qDot,qDotDes,u" << endl;
            }
        }

        ofstream fichier(("results_sinu/results_sinu.csv"),ios::out | ios::app);
        if(fichier)
        {   
            // for(int i=0;i<T;i++){
            //     fichier << xList[i](0,0) << "," << xList[i](1,0) << "," << uList[i](0,0) << endl;
            // } 
            fichier << xList[T](0,0) << "," <<  xDes[0] << "," << xList[T](1,0) << "," << xDes[1] << "," << uList[T-1](0,0) << endl;
            fichier.close();
            // fichier << "tau,q,qDot,theta_m,tau_ext,u" << endl;
            // for(int i=0;i<T;i++){
            //     fichier << xList[i](0,0) << "," << xList[i](1,0) << "," << xList[i](2,0) << "," << xList[i](3,0) << "," << 
            //     xList[i](4,0) << "," << uList[i](0,0) << endl;
            // } 
            // fichier << xList[T](0,0) << "," << xList[T](1,0) << "," << xList[T](2,0) << "," << xList[T](3,0) << "," << 
            // xList[T](4,0) << "," << uList[T-1](0,0) << endl;
            // fichier.close();
        }
        else{
            cerr << "erreur ouverte fichier" << endl;
        }
    }
   
    return 0;

}
