#include <iostream>
#include <fstream>

#include "config.h"

#include "lqrsolver.h"
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
    LQRSolver<double,4,1>::stateVec_t xinit,xDes,x;
    LQRSolver<double,4,1>::commandVec_t u;

    int i;
    unsigned int T = 1000;
    double dt=1e-3;
    unsigned int iterMax = 100;
    double stopCrit = 1e-5;
    LQRSolver<double,4,1>::stateVecTab_t xList;
    LQRSolver<double,4,1>::commandVecTab_t uList;
    LQRSolver<double,4,1>::traj lastTraj;

    RomeoSimpleActuator romeoActuatorModel(dt);
    RomeoSimpleActuator* romeoNoisyModel=NULL;
    CostRomeoPos costRomeoActuator;

    xinit << 0.0,0.0,0.0,0.0,0.0;
    xDes << 0.5,0.0,0.0,0.0;

    xinit[0] = -1.0;
    xinit[2] = xinit[0]*romeoActuatorModel.getR();

    LQRSolver<double,4,1> testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator);
    testSolverRomeoActuator.InitSolver(xinit,xDes,T,dt);

    int N = 100;
    gettimeofday(&tbegin,NULL);
    testSolverRomeoActuator.solveTrajectory();
    gettimeofday(&tend,NULL);

    lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
    xList = lastTraj.xList;
    uList = lastTraj.uList;


    texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    texec /= N;

    cout << endl;
    cout << "temps d'execution total du solveur ";
    cout << texec << endl;
    cout << "temps d'execution par pas de temps ";
    cout << texec/T << endl;

    ofstream fichier1("results1.csv",ios::out | ios::trunc);
    if(fichier1)
    {
        fichier1 << "tau,tauDot,q,qDot,u" << endl;
        x = xinit;
        fichier1 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                 << x(3, 0) << "," << uList[0] << endl;
        for (i = 1; i < T; i++)
        {
            x = romeoActuatorModel.computeNextState(dt, x, uList[i - 1]);
            fichier1 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                     << x(3, 0) << "," << uList[i] << endl;
        }
        fichier1 << xList[T](0, 0) << "," << xList[T](1, 0) << "," << xList[T](2, 0) << "," << xList[T](3, 0) << ","
                 << uList[T-1] << endl;
        fichier1.close();
    }
    else
        cerr << "erreur ouverte fichier" << endl;

    ofstream fichier2("results2.csv",ios::out | ios::trunc);
    if(fichier2)
    {
        fichier2 << "tau,tauDot,q,qDot,u" << endl;
        fichier2 << T << ',' << 1 << endl;
        for(int j=0;j<1;j++)
        {
            romeoNoisyModel = new RomeoSimpleActuator(dt,1);
            fichier2 << xList[0](0, 0) << "," << xList[0](1, 0) << "," << xList[0](2, 0) << "," << xList[0](3, 0) << ","
                     << uList[0](0, 0) << endl;
            x = xinit;
            for (i = 1; i < T; i++)
            {
                x = romeoNoisyModel->computeNextState(dt, x, uList[i - 1]);
                fichier2 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                         << x(3, 0) << "," << uList[i] << endl;
            }
            fichier2 << xList[T](0, 0) << "," << xList[T](1, 0) << "," << xList[T](2, 0) << "," << xList[T](3, 0) << ","
                     << uList[T-1] << endl;
            delete romeoNoisyModel;
        }
        fichier2.close();
    }
    else
        cerr << "erreur ouverte fichier" << endl;


    return 0;

}
