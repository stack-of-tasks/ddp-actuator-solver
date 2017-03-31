#include <iostream>
#include <fstream>

#include "config.h"

#include "ilqrsolver.h"
#include "awas.h"
#include "costVIA.h"

#include <time.h>
#include <sys/time.h>

using namespace std;
using namespace Eigen;

int main()
{
    struct timeval tbegin,tend;
    double texec=0.0;
    ILQRSolver<double,4,2>::stateVec_t xinit,xDes,x;
    ILQRSolver<double,4,2>::commandVec_t u;

    int i;
    unsigned int T = 3000;
    double dt=1e-3;
    unsigned int iterMax = 100;
    double stopCrit = 1e-5;
    ILQRSolver<double,4,2>::stateVecTab_t xList;
    ILQRSolver<double,4,2>::commandVecTab_t uList;
    ILQRSolver<double,4,2>::traj lastTraj;

    Awas awasModel(dt);
    Awas* awasNoisyModel=NULL;
    CostVIA costRomeoActuator;

    xinit << 0.0,0.0,0.1,0.0;
    xDes << 0.01,0.0,0.1,0.0;

    ILQRSolver<double,4,2> testSolverRomeoActuator(awasModel,costRomeoActuator,DISABLE_FULLDDP,DISABLE_QPBOX);
    testSolverRomeoActuator.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);

    int N = 1;
    gettimeofday(&tbegin,NULL);
    testSolverRomeoActuator.solveTrajectory();
    gettimeofday(&tend,NULL);

    lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
    xList = lastTraj.xList;
    uList = lastTraj.uList;
    unsigned int iter = lastTraj.iter;


    /*for(i=0;i<T-1;i++)
    {
        uList[i] <<0.1,0.0;
    }*/

    texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    texec /= N;

    cout << endl;
    cout << "temps d'execution total du solveur ";
    cout << texec << endl;
    cout << "temps d'execution par pas de temps ";
    cout << texec/T << endl;
    cout << "Nombre d'itérations : " << iter << endl;


    ofstream fichier1("results1.csv",ios::out | ios::trunc);
    if(fichier1)
    {
        fichier1 << "tau,tauDot,q,qDot,u1,u2" << endl;
        x = xinit;
        fichier1 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                 << x(3, 0) << "," << uList[0](0,0) << "," << uList[0](1,0) << endl;
        for (i = 1; i < T; i++)
        {
            x = awasModel.computeNextState(dt, x, uList[i - 1]);
            fichier1 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                     << x(3, 0) << "," << uList[i - 1](0,0) << "," << uList[i - 1](1,0) << endl;
        }
        fichier1 << xList[T](0, 0) << "," << xList[T](1, 0) << "," << xList[T](2, 0) << "," << xList[T](3, 0) << ","
                 << uList[T - 1](0, 0)<< "," << uList[T-1](1,0) << endl;
        fichier1.close();
    }
    else
        cerr << "erreur ouverte fichier" << endl;

    ofstream fichier2("results2.csv",ios::out | ios::trunc);
    if(fichier2)
    {
        fichier2 << "tau,tauDot,q,qDot,u1,u2" << endl;
        fichier2 << T << ',' << 0 << endl;
        for(int j=0;j<0;j++)
        {
            awasNoisyModel = new Awas(dt,1);
            fichier2 << xList[0](0, 0) << "," << xList[0](1, 0) << "," << xList[0](2, 0) << "," << xList[0](3, 0) << ","
                     << uList[0](0, 0) << ',' << uList[0](1, 0) << endl;
            x = xinit;
            for (i = 1; i < T; i++)
            {
                x = awasNoisyModel->computeNextState(dt, x, uList[i - 1]);
                fichier2 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                         << x(3, 0) << "," << uList[i-1](0,0)<< "," << uList[i-1](1,0) <<endl;
            }
            fichier2 << xList[T](0, 0) << "," << xList[T](1, 0) << "," << xList[T](2, 0) << "," << xList[T](3, 0) << ","
                     << uList[T - 1](0,0) << "," << uList[T-1](1,0) << endl;
            delete awasNoisyModel;
        }
        fichier2.close();
    }
    else
        cerr << "erreur ouverte fichier" << endl;


    return 0;

}
