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
    bench.sendPositionCommand(800.0*M_PI/2048.0);
    sleep(1);
    bench.sendStopCommand();

    ILQRSolver<double,4,1>::stateVec_t x,x_offset,xinit,xDes,xStore;
    ILQRSolver<double,4,1>::commandVec_t u;
    double jointDes=0;

    std::vector<ILQRSolver<double,4,1>::stateVec_t> xTab;
    std::vector<ILQRSolver<double,4,1>::commandVec_t> uTab;


    cout << "begin" << endl;

    double dt=1e-3;
    unsigned int iterMax = 100;
    double stopCrit = 1e-5;
    ILQRSolver<double,4,1>::stateVecTab_t xList;
    ILQRSolver<double,4,1>::commandVecTab_t uList;
    ILQRSolver<double,4,1>::traj lastTraj;
    BenchCom::fullState full_state;

    RomeoSimpleActuator romeoActuatorModel(dt);
    CostRomeoPos costRomeoActuator;
    ILQRSolver<double,4,1> testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator,DISABLE_FULLDDP,ENABLE_QPBOX);

    testSolverRomeoActuator.FirstInitSolver(xinit,xDes,N,dt,iterMax,stopCrit);

    full_state = bench.getStateFromSerial();
    xinit << full_state.jointPos,0.0,full_state.motorPos,0.0;
    x_offset << 0.0,0.0,+xinit[2] - xinit[0]*95.0,0.0;
    xinit = xinit - x_offset;
    xDes << 1100.0*(M_PI/2048.0),0.0,0.0,0.0;

    cout << "xInit : " << xinit.transpose() << endl;
    cout << "xDes : " << xDes.transpose() << endl;


    //xStore << full_state.jointPosInt,0.0,full_state.motorPosInt,0.0;
    //xTab.push_back(xStore);

    testSolverRomeoActuator.initSolver(xinit, xDes);
    testSolverRomeoActuator.solveTrajectory();
    lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();

    for(i=0;i<N;i++)
    {
        current = -lastTraj.uList[i](0,0);
        u << current;
        jointDes = xDes[0];
        full_state = bench.sendCurrentCommand(current,jointDes);

        x << full_state.jointPos,full_state.jointVel,full_state.motorPos,full_state.motorVel;

        xinit = x-x_offset;

        xStore = x ;
        xTab.push_back(xStore);
        //xTab.push_back(xinit);
        uTab.push_back(u);
    }

    bench.sendStopCommand();

    ofstream fichier1("resultsBench.csv",ios::out | ios::trunc);
    fichier1 << "q,qDot,tau,tauDot,u" << endl;
    if(fichier1)
    {
        for (i = 0; i < N; i++)
        {
            x = xTab[i];
            fichier1 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                     << x(3, 0) << "," << uTab[i] << endl;
        }
        fichier1 << xTab[N](0, 0) << "," << xTab[N](1, 0) << "," << xTab[N](2, 0) << "," << xTab[N](3, 0) << ","
                 << uTab[N - 1](0, 0) << endl;
        fichier1.close();
    }
    else
        cerr << "erreur ouverte fichier" << endl;

    ofstream fichier2("resultsBenchMPC.csv",ios::out | ios::trunc);
    fichier2 << "q,qDot,tau,tauDot,u" << endl;
    if(fichier2)
    {
        for(i=0;i<N;i++)
        {

            x = lastTraj.xList[i];
            fichier2 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                     << x(3, 0) << "," << -lastTraj.uList[i] << endl;

        }
        x = lastTraj.xList[N+1];
        fichier2 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                 << x(3, 0) << "," << -lastTraj.uList[N] << endl;
    }
    else
        cerr << "erreur ouverte fichier" << endl;

    return 0;
}
