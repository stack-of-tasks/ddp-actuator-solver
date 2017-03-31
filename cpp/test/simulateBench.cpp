#include <iostream>
#include <fstream>
#include <stdint.h>
#include <unistd.h>

#include <math.h>

#include "config.h"

#include "ilqrsolver.h"
#include "romeosimpleactuator.h"
#include "costRomeoPos.h"
#include "BenchCom.h"

#include <fcntl.h>
#include <termios.h>

#include <sys/time.h>

using namespace std;
using namespace Eigen;

struct timeval current_time,last_time;

uint8_t crc,send,received;
uint16_t posTarget;
uint8_t requiredJointposH;
uint8_t requiredJointposL;
uint8_t requiredSpeedH;
uint8_t requiredSpeedL;
uint8_t requiredCurrentH;
uint8_t requiredCurrentL;
uint8_t requiredPwmH;
uint8_t requiredPwmL;
uint8_t motorH;
uint8_t motorL;
int8_t motorTurn;
uint8_t jointH;
uint8_t jointL;
uint8_t currentH;
uint8_t currentL;
uint8_t pwmH;
uint8_t pwmL;
uint8_t crc1;
uint8_t crc2;
uint8_t crcCalc1;
uint8_t crcCalc2;

int32_t motorPosInt;
uint16_t jointPosInt;
uint32_t currentInt;

double motorPos;
double jointPos;
double motorPosOld;
double jointPosOld;
double motorVel;
double jointVel;
double diffAngle;
double diff_time;
double current;

double current_offset=0.0;

int main()
{
    int i;
    unsigned int N = 3000;
    struct timeval tbegin,tend;

    double texec=0.0;

    BenchCom bench;
    cout << "current offset : " << bench.getCurrentOffset() << endl;
    bench.sendPositionCommand(1000.0*M_PI/2048.0);
    sleep(1);
    bench.sendStopCommand();



    ofstream fichier("resultsBench.csv",ios::out | ios::trunc);
    fichier << "tau,tauDot,q,qDot,u" << endl;
    ILQRSolver<double,4,1>::stateVec_t x,x_offset,xinit,xDes;
    ILQRSolver<double,4,1>::commandVec_t u;
    double jointDes=0;

    std::vector<ILQRSolver<double,4,1>::stateVec_t> xTab;
    std::vector<ILQRSolver<double,4,1>::stateVec_t> xDesTab;
    std::vector<ILQRSolver<double,4,1>::commandVec_t> uTab;
    std::vector<ILQRSolver<double,4,1>::traj> trajTab;
    BenchCom::fullState full_state;

    cout << "begin" << endl;


    full_state = bench.getStateFromSerial();
    double joint_offset = bench.getJointOffset();
    double motor_offset = bench.getMotorOffset();
    x << full_state.jointPos-joint_offset,full_state.jointVel,full_state.motorPos-motor_offset,full_state.motorVel;


    xinit << x;
    xDes << xinit(0,0)+ 0.5,0.0,0.0,0.0;

    unsigned int T = 500;
    double dt=1e-3;
    unsigned int iterMax = 100;
    double stopCrit = 1e-5;
    ILQRSolver<double,4,1>::stateVecTab_t xList;
    ILQRSolver<double,4,1>::commandVecTab_t uList;
    ILQRSolver<double,4,1>::traj lastTraj;

    RomeoSimpleActuator romeoActuatorModel(dt);
    CostRomeoPos costRomeoActuator;
    ILQRSolver<double,4,1> testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator,DISABLE_FULLDDP,ENABLE_QPBOX);
    testSolverRomeoActuator.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);

	cout << "xInit : " << xinit.transpose() << endl;
    cout << "xDes : " << xDes.transpose() << endl;
    cout << xDes(0,0)<<endl;
    xTab.push_back(xinit);

    for(i=0;i<N;i++)
    {
        if(i>N/2) xDes << 0.0,0.0,0.0,0.0;
        jointDes = xDes(0,0);

        testSolverRomeoActuator.initSolver(xinit, xDes);
        testSolverRomeoActuator.solveTrajectory();
        lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();

        u = -lastTraj.uList[0];
        //full_state = bench.sendCurrentCommand(u(0,0),jointDes);
        full_state = bench.SimulatesendCurrentCommand(full_state,u(0,0),jointDes);
        x << full_state.jointPos/*-joint_offset*/,full_state.jointVel,full_state.motorPos/*-motor_offset*/,full_state.motorVel;

        xinit = x;

        xTab.push_back(xinit);
        xDesTab.push_back(xDes);
        uTab.push_back(u);
        trajTab.push_back(lastTraj);

        //cout << xinit[0] << " - " << xDes[0] << endl;

        //cout << i << " : " << xinit.transpose() << endl;
    }

    /*while(1)
    {
        testSolverRomeoActuator.initSolver(xinit, xDes);
        testSolverRomeoActuator.solveTrajectory();
        lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();


        jointDes = (xDes[0] + x_offset[0])*2048.0/M_PI;
        u = -lastTraj.uList[0];
        cout << -u << endl;
        x = sendCurrentCommand(ser,u,jointDes);

        xinit = x-x_offset;

        //cout << xinit.transpose() << endl;
    }*/


    bench.sendStopCommand();

    ofstream fichier1("resultsBench.csv",ios::out | ios::trunc);
    fichier1 << "q,qDot,tau,tauDot,u" << endl;
    if(fichier1)
    {
        for (i = 0; i < N; i++)
        {
            x = xTab[i];
            fichier1 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                     << x(3, 0) << "," << uTab[i] << "," << xDesTab[i](0.0) << endl;
        }
        fichier1 << xTab[T](0, 0) << "," << xTab[T](1, 0) << "," << xTab[T](2, 0) << "," << xTab[T](3, 0) << ","
                 << uTab[T - 1](0, 0) << "," << xDesTab[i](0.0) << endl;
        fichier1.close();
    }
    else
        cerr << "erreur ouverte fichier" << endl;

    ofstream fichier2("resultsBenchMPC.csv",ios::out | ios::trunc);
    fichier2 << "q,qDot,tau,tauDot,u" << endl;
    fichier2 << T << ',' << N << endl;
    if(fichier2)
    {
        for(i=0;i<N;i++)
        {
            for(int j =0;j<=T;j++)
            {
                x = trajTab[i].xList[j];
                fichier2 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                         << x(3, 0) << "," << -trajTab[i].uList[j] << endl;
            }
            x = trajTab[i].xList[T+1];
            fichier2 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                     << x(3, 0) << "," << -trajTab[i].uList[T] << endl;
        }
    }
    else
        cerr << "erreur ouverte fichier" << endl;
    return 0;

}
