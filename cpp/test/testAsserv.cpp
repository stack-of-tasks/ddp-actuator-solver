#include <unistd.h>
#include <iostream>
#include <fstream>

#include "ilqrsolver.h"
#include "romeosimpleactuator.h"
#include "costRomeoPos.h"

#include <time.h>
#include <sys/time.h>


using namespace std;
using namespace Eigen;

int main()
{
    struct timeval tbegin,tend;
    double texec=0.0;
    ILQRSolver<double,4,1>::stateVec_t xinit,xDes,x;
    ILQRSolver<double,4,1>::commandVec_t u;

    int i;
    unsigned int T = 1000;
    double dt=1e-3;
    unsigned int iterMax = 100;
    double stopCrit = 1e-5;
    ILQRSolver<double,4,1>::stateVecTab_t xList;
    ILQRSolver<double,4,1>::commandVecTab_t uList;
    ILQRSolver<double,4,1>::traj lastTraj;
    ILQRSolver<double,4,1>::commandR_stateC_t fbGain;

    RomeoSimpleActuator romeoActuatorModel(dt);
    RomeoSimpleActuator romeoNoisyModel(dt,1);
    CostRomeoPos costRomeoActuator;

    fbGain << 0.1,0.0,0.0,0.0;

    xinit << 0.0,0.0,0.0,0.0;
    xDes << 1.0,0.0,0.0,0.0;
    x = xinit;

    xinit[0] = 0.0;
    xinit[2] = xinit[0]*romeoActuatorModel.getR();

    ILQRSolver<double,4,1> testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator,DISABLE_FULLDDP,ENABLE_QPBOX);
    testSolverRomeoActuator.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);

    testSolverRomeoActuator.solveTrajectory();
    lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
    xList = lastTraj.xList;
    uList = lastTraj.uList;

    xList[i] = x;

    for(int i=0;i<T;i++)
    {
        u = uList[0] - fbGain*(x-xDes);
        x = romeoNoisyModel.computeNextState(dt,x,u);
    }


    ofstream fichier("results.csv",ios::out | ios::trunc);
    if(fichier)
    {
        fichier << "tau,tauDot,q,qDot,u" << endl;
        for(int i=0;i<T;i++) fichier << xList[i](0,0) << "," << xList[i](1,0) << "," << xList[i](2,0) << "," << xList[i](3,0) << "," << uList[i](0,0) << endl;
        fichier.close();
    }
    else
        cerr << "erreur ouverte fichier" << endl;
    return 0;

}
