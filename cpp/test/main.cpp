#include <iostream>
#include <fstream>
#include <string>

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
    struct timeval tbegin,tend;
    double texec=0.0;
    ILQRSolver<double,4,1>::stateVec_t xinit,xDes,x;
    ILQRSolver<double,4,1>::commandVec_t u;
    ILQRSolver<double,4,1>::commandR_stateC_tab_t Ktab;
    ILQRSolver<double,4,1>::commandR_stateC_t K;
    std::vector<double> desList;
    double lastDes;

    int i;
    unsigned int M;
    double dt=1e-3;
    unsigned int iterMax = 100;
    double stopCrit = 1e-5;
    ILQRSolver<double,4,1>::stateVecTab_t xList;
    ILQRSolver<double,4,1>::commandVecTab_t uList;
    ILQRSolver<double,4,1>::traj lastTraj;

    RomeoSimpleActuator romeoActuatorModel(dt);
    RomeoSimpleActuator* noisyModel=NULL;
    CostRomeoPos cost;

    bool readTrajFile = false;
    if(readTrajFile)
    {
        M = 0;
        string ligne;
        ifstream trajFile("/home/flo/climbingKnee.csv", ios::in);
        if (trajFile) {
            while (getline(trajFile, ligne))  // tant que l'on peut mettre la ligne dans "contenu"
            {
                M++;
                desList.push_back(atof(ligne.c_str()));
                lastDes = atof(ligne.c_str());
            }
        } else cerr << "Impossible d'ouvrir le fichier !" << endl;
        for(i=0;i<1000;i++) desList.push_back(lastDes);
    }
    else
    {
        M = 6000;
        for (i = 0; i < M; i++) {

            if (i > 1600) {
                desList.push_back(0.0);
            } else {
                if (i > 650) {
                    desList.push_back(1.0);
                } else {
                    desList.push_back(0.0);
                }
            }
        }
    }

    xinit << 0.0,0.0,0.0,0.0;
    xDes << 0.0,0.0,0.0,0.0;

    ILQRSolver<double,4,1> testSolverRomeoActuator(romeoActuatorModel,cost,DISABLE_FULLDDP,DISABLE_QPBOX);


    bool MPC = true;

    if(!MPC)
    {
        xDes << 0.5,0.0,0.0,0.0;
        unsigned int T = 1000;
        testSolverRomeoActuator.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);

        int N = 1;
        gettimeofday(&tbegin,NULL);
        testSolverRomeoActuator.solveTrajectory();
        gettimeofday(&tend,NULL);

        lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
        xList = lastTraj.xList;
        uList = lastTraj.uList;
        unsigned int iter = lastTraj.iter;

        Ktab = testSolverRomeoActuator.getLastSolvedTrajectoryGains();


        texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
        texec /= N;

        cout << endl;
        cout << "temps d'execution total du solveur ";
        cout << texec << endl;
        cout << "temps d'execution par pas de temps ";
        cout << texec/T << endl;
        cout << "Nombre d'itérations : " << iter << endl;


        ofstream fichier1("resultsDC1.csv",ios::out | ios::trunc);
        if(fichier1)
        {
            fichier1 << "tau,tauDot,q,qDot,u,K" << endl;
            x = xinit;
            fichier1 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                     << uList[0] << "," << Ktab[0](0,0) <<endl;
            for (i = 1; i < T; i++)
            {
                x = romeoActuatorModel.computeNextState(dt, x, uList[i - 1]);
                fichier1 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                         << uList[i - 1] << "," << Ktab[i-1](0,0) << endl;
            }
            fichier1 << xList[T](0, 0) << "," << xList[T](1, 0) << "," << xList[T](2, 0) << ","
                     << uList[T - 1](0, 0)<< "," << Ktab[T-1](0,0) << endl;
            fichier1.close();
        }
        else
            cerr << "erreur ouverte fichier" << endl;

        ofstream fichier2("resultsDC2.csv",ios::out | ios::trunc);
        if(fichier2)
        {
            fichier2 << "tau,tauDot,q,qDot,u" << endl;
            fichier2 << T << ',' << 2 << endl;
            for(int j=0;j<2;j++)
            {
                noisyModel = new RomeoSimpleActuator(dt,1);
                fichier2 << xList[i](0, 0) << "," << xList[i](1, 0) << "," << xList[i](2, 0) << ","
                         << uList[i](0, 0) << endl;
                x = xinit;
                for (i = 1; i < T; i++)
                {
                    x = noisyModel->computeNextState(dt, x, uList[i - 1]);
                    fichier2 << x(0, 0) << "," << x(1, 0) << "," << x(2, 0) << ","
                             << uList[i - 1] << endl;
                }
                fichier2 << xList[T](0, 0) << "," << xList[T](1, 0) << "," << xList[T](2, 0) << ","
                         << uList[T - 1](0, 0) << endl;
                delete noisyModel;
            }
            fichier2.close();
        }
        else
            cerr << "erreur ouverte fichier" << endl;
    }
    else
    {
        noisyModel = new RomeoSimpleActuator(dt,0);
        unsigned int T = 100;
        testSolverRomeoActuator.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);

        ofstream fichier1("resultsDC1.csv",ios::out | ios::trunc);
        ofstream fichier2("resultsDC2.csv",ios::out | ios::trunc);
        fichier1 << "tau,tauDot,q,u,K" << endl;
        fichier2 << "tau,tauDot,q,u,K" << endl;
        fichier2 << M << ',' << 1 << endl;


        xinit << 0.0,0.0,0.0,0.0;
        xDes << desList[0],0.0,0.0,0.0;
        fichier1 << xinit(0,0) << "," << xinit(1,0) << "," << xinit(2,0) << "," << 0.0 << "," << 0.0 << endl;
        for(int i=0;i<M;i++)
        {
            xDes[0] = desList[i+(T/2)];
            testSolverRomeoActuator.initSolver(xinit,xDes);
            testSolverRomeoActuator.solveTrajectory();
            lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
            Ktab = testSolverRomeoActuator.getLastSolvedTrajectoryGains();
            xList = lastTraj.xList;
            uList = lastTraj.uList;
            K = Ktab[0];
            u << uList[0](0);// - K(0,0)*(xDes[0]-xinit[0]);
            xinit = noisyModel->computeNextState(dt,xinit,u);

            fichier1 << xinit(0,0) << "," << xinit(1,0) << "," << xinit(2,0) << "," << u << "," << K(0,0) << endl;

        }
        xinit << 0.0,0.0,0.0,0.0;
        xDes << desList[0],0.0,0.0,0.0;
        fichier1 << xinit(0,0) << "," << xinit(1,0) << "," << xinit(2,0) << "," << 0.0 << "," << 0.0 << endl;
        for(int i=0;i<M;i++)
        {
            xDes[0] = desList[i+(T/2)];
            testSolverRomeoActuator.initSolver(xinit,xDes);
            testSolverRomeoActuator.solveTrajectory();
            lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
            Ktab = testSolverRomeoActuator.getLastSolvedTrajectoryGains();
            xList = lastTraj.xList;
            uList = lastTraj.uList;
            K = Ktab[0];
            u << - K*(xDes-xinit);
            xinit = noisyModel->computeNextState(dt,xinit,u);

            fichier1 << xinit(0,0) << "," << xinit(1,0) << "," << xinit(2,0) << "," << u << "," << K(0,0) << endl;

        }
        xinit << 0.0,0.0,0.0,0.0;
        xDes << desList[0],0.0,0.0,0.0;
        fichier1 << xinit(0,0) << "," << xinit(1,0) << "," << xinit(2,0) << "," << 0.0 << "," << 0.0 << endl;
        for(int i=0;i<M;i++)
        {
            xDes[0] = desList[i+(T/2)];
            testSolverRomeoActuator.initSolver(xinit,xDes);
            testSolverRomeoActuator.solveTrajectory();
            lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
            Ktab = testSolverRomeoActuator.getLastSolvedTrajectoryGains();
            xList = lastTraj.xList;
            uList = lastTraj.uList;
            K = Ktab[0];
            u << uList[0](0) - K*(xDes-xinit);
            xinit = noisyModel->computeNextState(dt,xinit,u);

            fichier1 << xinit(0,0) << "," << xinit(1,0) << "," << xinit(2,0) << "," << u << "," << K(0,0) << endl;

        }
        delete noisyModel;
        for(int j=0;j<=2;j++)
        {
            cout << j << "/2" << endl;
            xinit << 0.0,0.0,0.0,0.0;
            xDes << desList[0],0.0,0.0,0.0;
            fichier2 << xinit(0, 0) << "," << xinit(1, 0) << "," << xinit(2, 0) << "," << 0.0 << "," << 0.0
                     << endl;
            noisyModel = new RomeoSimpleActuator(dt,1);
            for (int i = 0; i < M; i++)
            {
                xDes[0] = desList[i+(T/2)];
                testSolverRomeoActuator.initSolver(xinit, xDes);
                testSolverRomeoActuator.solveTrajectory();
                lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
                Ktab = testSolverRomeoActuator.getLastSolvedTrajectoryGains();
                xList = lastTraj.xList;
                uList = lastTraj.uList;
                K = Ktab[0];
                u << uList[0](0);
                xinit = noisyModel->computeNextState(dt, xinit, u);

                fichier2 << xinit(0, 0) << "," << xinit(1, 0) << "," << xinit(2, 0) << "," << u << "," << K(0, 0)
                         << endl;
            }
            xinit << 0.0,0.0,0.0,0.0;
            xDes << desList[0],0.0,0.0,0.0;
            fichier2 << xinit(0, 0) << "," << xinit(1, 0) << "," << xinit(2, 0) << "," << 0.0 << "," << 0.0
                     << endl;
            for (int i = 0; i < M; i++)
            {
                xDes[0] = desList[i+(T/2)];
                testSolverRomeoActuator.initSolver(xinit, xDes);
                testSolverRomeoActuator.solveTrajectory();
                lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
                Ktab = testSolverRomeoActuator.getLastSolvedTrajectoryGains();
                xList = lastTraj.xList;
                uList = lastTraj.uList;
                K = Ktab[0];
                u << - K*(xDes-xinit);
                xinit = noisyModel->computeNextState(dt, xinit, u);

                fichier2 << xinit(0, 0) << "," << xinit(1, 0) << "," << xinit(2, 0) << "," << u << "," << K(0, 0)
                         << endl;
            }
            xinit << 0.0,0.0,0.0,0.0;
            xDes << desList[0],0.0,0.0,0.0;
            fichier2 << xinit(0, 0) << "," << xinit(1, 0) << "," << xinit(2, 0) << "," << 0.0 << "," << 0.0
                     << endl;
            for (int i = 0; i < M; i++)
            {
                xDes[0] = desList[i+(T/2)];
                testSolverRomeoActuator.initSolver(xinit, xDes);
                testSolverRomeoActuator.solveTrajectory();
                lastTraj = testSolverRomeoActuator.getLastSolvedTrajectory();
                Ktab = testSolverRomeoActuator.getLastSolvedTrajectoryGains();
                xList = lastTraj.xList;
                uList = lastTraj.uList;
                K = Ktab[0];
                u << uList[0](0) - K*(xDes-xinit);
                xinit = noisyModel->computeNextState(dt, xinit, u);

                fichier2 << xinit(0, 0) << "," << xinit(1, 0) << "," << xinit(2, 0) << "," << u << "," << K(0, 0)
                         << endl;
            }
            delete noisyModel;
        }
        fichier1.close();
        fichier2.close();
    }
    return 0;

}
