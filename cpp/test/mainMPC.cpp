#include <iostream>
#include <fstream>

#include "config.h"

#include "ilqrsolver.h"
#include "romeosimpleactuator.h"
#include "costRomeoPos.h"

#include <time.h>
#include <sys/time.h>


using namespace std;
using namespace Eigen;

int main()
{
    cout << endl;
    struct timeval tbegin,tend;
    double texec=0.0;
    ILQRSolver<double,4,1>::stateVec_t xinit,xDes;
    ILQRSolver<double,4,1>::commandVec_t u;
    ILQRSolver<double,4,1>::commandR_stateC_tab_t Ktab;
    ILQRSolver<double,4,1>::commandR_stateC_t K;

    unsigned int T = 100;
    unsigned int M = 3200;
    double dt=1e-3;
    unsigned int iterMax = 100;
    double stopCrit = 1e-3;
    ILQRSolver<double,4,1>::stateVecTab_t xList;
    ILQRSolver<double,4,1>::commandVecTab_t uList;
    ILQRSolver<double,4,1>::traj lastTraj;

    srand(time(NULL));

    RomeoSimpleActuator romeoActuatorModel(dt);
    RomeoSimpleActuator romeoNoisyModel(dt,1);
    CostRomeoPos costRomeoActuator;

    xinit << 0.0,0.0,0.0,0.0;
    xDes << 0.0,0.0,0.0,0.0;

    xinit[0] = 0.0;
    xinit[2] = xinit[0]*romeoActuatorModel.getR();

    ILQRSolver<double,4,1> testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator,DISABLE_FULLDDP,ENABLE_QPBOX);



    ofstream fichier("resultsMPC.csv",ios::out | ios::trunc);
    if(!fichier)
    {
        cerr << "erreur fichier ! " << endl;
        return 1;
    }
    fichier << T << "," << M << endl;
    fichier << "tau,tauDot,q,qDot,u" << endl;

    ofstream fichierF("resultsMPCF.csv",ios::out | ios::trunc);
    if(!fichierF)
    {
        cerr << "erreur fichier ! " << endl;
        return 1;
    }
    fichierF << "tau,tauDot,q,qDot,u,k" << endl;

    fichierF << xinit(0,0) << "," << xinit(1,0) << "," << xinit(2,0) << "," << xinit(3,0) << "," << 0.0 << "," << 0.0 << endl;

    testSolverRomeoActuator.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);

    gettimeofday(&tbegin,NULL);
    for(int i=0;i<M;i++)
    {
        if(i>200) xDes[0] = 1.0;
        if(i>1700) xDes[0] = 0.5;
        testSolverRomeoActuator.initSolver(xinit,xDes);
        testSolverRomeoActuator.solveTrajectory();
        lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
        Ktab = testSolverRomeoActuator.getLastSolvedTrajectoryGains();
        xList = lastTraj.xList;
        uList = lastTraj.uList;
        K = Ktab[0];
        u << uList[0](0) - K(0,0)*(xDes[0]-xinit[0]);
        xinit = romeoNoisyModel.computeNextState(dt,xinit,u);

        for(int j=0;j<T;j++) fichier << xList[j](0,0) << "," << xList[j](1,0) << "," << xList[j](2,0) << "," << xList[j](3,0) << "," << uList[j](0,0) << endl;
        fichier << xList[T](0,0) << "," << xList[T](1,0) << "," << xList[T](2,0) << "," << xList[T](3,0) << "," << 0.0 << endl;

        fichierF << xinit(0,0) << "," << xinit(1,0) << "," << xinit(2,0) << "," << xinit(3,0) << "," << u << "," << K(0,0) << endl;
    }
    gettimeofday(&tend,NULL);


    texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    texec = (double)(tend.tv_usec - tbegin.tv_usec);

    cout << "temps d'execution total du solveur ";
    cout << texec/1000000.0 << endl;
    cout << "temps d'execution par pas de MPC ";
    cout << texec/(T*1000000) << endl;

    fichier.close();
    fichierF.close();

    return 0;
}

