#include <iostream>
#include <fstream>
#include <string>

#include "config.h"

#include "ilqrsolver.h"
#include "romeosimpleactuator.h"
#include "strainwaveactuator.h"
#include "costRomeoPos.h"
#include "costRomeoTorque.h"

#include <time.h>
#include <sys/time.h>

#define N 40000
#define CURVES 7

#define sign(x) (x>0.0)?(1.0):((x==0.0)?(0.0):(-1.0))

using namespace std;
using namespace Eigen;

int main()
{
    struct timeval tbegin,tend;
    double texec=0.0;
    ILQRSolver<double,4,1>::stateVec_t xinit,xDes,x;
    ILQRSolver<double,4,1>::commandVec_t u;
    ILQRSolver<double,4,1>::commandR_stateC_tab_t Ktab;
    ILQRSolver<double,4,1>::commandR_stateC_t K;
    vector<double> desList(N);
    vector<vector<double> > storage(N, vector<double>(CURVES));
    double lastDes;

    int i,j;
    double dt=1e-3;
    unsigned int iterMax = 100;
    double stopCrit = 1e-5;
    ILQRSolver<double,4,1>::stateVecTab_t xList;
    ILQRSolver<double,4,1>::commandVecTab_t uList;
    ILQRSolver<double,4,1>::traj lastTraj;

    RomeoSimpleActuator romeoActuatorModel(dt);
    RomeoSimpleActuator vsaActuator(dt,8);
    StrainWaveActuator strainwaveactuator(dt);
    CostRomeoPos cost;
    ILQRSolver<double,4,1> testSolverRomeoActuator(romeoActuatorModel,cost,DISABLE_FULLDDP,ENABLE_QPBOX);

    double f_0 = 0.1;
    double k = 1.1;
    double t=0.0;

    for (i = 0; i < N; i++)
    {
        t = (double)i/1000.0;
        desList[i] = 1.0*sin(2.0*M_PI*(f_0*t+(k/2.0)*t*t));
        //desList[i] = 1.0*sin(2.0*M_PI*f_0*((pow(k,t)-1)/log(k)));
        storage[i][5] = (f_0+k*t);
        //storage[i][5] = f_0*(pow(k,t));
        if((desList[i]>0.9999)&&(desList[i]<1.0001))
        {
            storage[i][4] = 1.0;
        }
        else
        {
            if ((desList[i]<-0.9999)&&(desList[i]>-1.0001))
            {
                storage[i][4] = -1.0;
            }
            else
            {
                storage[i][4] = 0.0;
            }
        }
        //desList[i] = 1.0;
        //if(i<1000) desList[i] = 0.0;
    }
    x << 0.0,0.0,0.0,0.0;
    xDes << 0.0, desList[0],0.0,0.0;
    double err = 0.0;
    double err_int = 0.0;
    double err_int_int = 0.0;
    unsigned int T = 30;
    testSolverRomeoActuator.FirstInitSolver(x,xDes,T,dt,iterMax,stopCrit);

    for(i=0;i<N;i++)
    {
        //xDes << 0.0, desList[i],0.0,0.0;
        xDes << desList[i],0.0,0.0,0.0;
        testSolverRomeoActuator.initSolver(x,xDes);
        testSolverRomeoActuator.solveTrajectory();
        lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
        err = desList[i] - x(0,0);
        u << lastTraj.uList[0];
        u << 2.0*(err);
        x = romeoActuatorModel.computeNextState(dt, x, u);

        storage[i][0] = desList[i];
        storage[i][1] = x[0];
    }

    u << 0.0;
    ILQRSolver<double,4,1>::stateVec_t x_VSA;
    x_VSA << 0.0,0.0,0.0,0.0;

    for(i=0;i<N;i++)
    {
        err = desList[i] - x_VSA(0,0);
        err_int += dt*err;
        u << 0.4*err + 0.0*err_int;
        //u << 1.0;
        if(u(0)>5.0) u << 5.0;
        if(u(0)<-5.0) u << -5.0;
        x_VSA = vsaActuator.computeNextState(dt, x_VSA, u);
        storage[i][2] = x_VSA(0,0);
    }

    double tau=0.0;
    u << 0.0;
    ILQRSolver<double,2,1>::stateVec_t x_strain;
    x_strain << 0.0,0.0;
    for(i=0;i<N;i++)
    {
        tau = strainwaveactuator.J*(strainwaveactuator.computeStateDeriv(dt,x_strain,u)(1,0));
        err = desList[i] - tau;
        err_int += dt*err;
        err_int_int += dt*err_int;
        u << /*0.05*err +*/ 70.0*err_int + 50.0*err_int_int;
        if(u(0)>5.0) u << 5.0;
        if(u(0)<-5.0) u << -5.0;
        x_strain = strainwaveactuator.computeNextState(dt, x_strain, u);
        storage[i][3] = 0.0;
    }


    ofstream fichier1("results.csv",ios::out | ios::trunc);
    if(fichier1)
    {
        fichier1 << "t,des,u,q,tau,q_dot,tau_dot" << endl;
        fichier1 << N << "," << CURVES << endl;
        for (i = 0; i < N; i++)
        {
            fichier1 << (double)i/1000.0 << ",";
            for(j=0;j<CURVES;j++) fichier1 << storage[i][j] << ",";
            fichier1 <<endl;
        }
        fichier1.close();
    }
    else
        cerr << "erreur ouverte fichier" << endl;
    return 0;

}
