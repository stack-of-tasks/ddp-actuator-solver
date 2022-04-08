#include <sys/time.h>
#include <time.h>

#include <ddp-actuator-solver/ddpsolver.hh>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "ddp-actuator-solver/pyrene_actuator/pyreneActuator.hh"
#include "ddp-actuator-solver/pyrene_actuator/pyreneCostFunction.hh"

using namespace std;
using namespace Eigen;

vector<double> fillVector(string repoBags, string fileName) {
  vector<double> fillVector(2000);
  ifstream file((repoBags + fileName).c_str(), ios::in);
  if (!file) {
    cerr << "File " << fileName.c_str() << " not Read" << endl;
  }

  double val;
  for (int j = 0; j < 2000; j++) {
    string line;
    getline(file, line);
    stringstream ss(line);
    if (ss >> val) {
      fillVector[j] = val;
    }
  }
  return fillVector;
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    cerr << "Convention: mainBags nameRepoBags " << endl;
    return 1;
  }
  string repoBags = argv[1];

  vector<double> vec_joint_pos = fillVector(repoBags, "joint_pos.txt");

  struct timeval tbegin, tend;
  double texec = 0.0;
  DDPSolver<double, 2, 1>::stateVec_t xinit, xDes, x;
  DDPSolver<double, 2, 1>::commandVec_t u;

  unsigned int T = 50;
  double dt = 1e-3;
  unsigned int iterMax = 10;
  double stopCrit = 1e-3;  // 0.01;
  DDPSolver<double, 2, 1>::stateVecTab_t xList;
  DDPSolver<double, 2, 1>::commandVecTab_t uList;
  DDPSolver<double, 2, 1>::traj lastTraj;

  pyreneActuator pyreneActuator;
  CostFunctionPyreneActuator costFunction;
  costFunction.setTauLimit(70);
  costFunction.setJointLimit(0.0, -2.35619449019);
  costFunction.setJointVelLimit(30.0, -30.0);
  // CostFunction<double,2,1>::stateMat_t Q;
  // Q << 500.0,0.0,0.0,0.01;
  // CostFunction<double,2,1>::commandMat_t P;
  // P << 100.0;
  // costFunction.setCostGainState(Q);
  // costFunction.setCostGainTorqueConstraint(P);
  // pyreneActuator.setLoadParam(30.0,-0.021481595, 0.10);
  DDPSolver<double, 2, 1> testSolverActuator(pyreneActuator, costFunction,
                                             DISABLE_FULLDDP, DISABLE_QPBOX);

  double dx_joint;
  dx_joint = 0.5422;
  xinit << vec_joint_pos[0], dx_joint;
  xDes << vec_joint_pos[1], 0.0;
  unsigned int nbIterTestMax = 2000.0;
  unsigned int iter;
  testSolverActuator.FirstInitSolver(xinit, xDes, T, dt, iterMax, stopCrit);

  for (unsigned int i = 0; i < nbIterTestMax - 1; i++) {
    gettimeofday(&tbegin, NULL);

    testSolverActuator.initSolver(xinit, xDes);
    testSolverActuator.solveTrajectory();
    lastTraj = testSolverActuator.getLastSolvedTrajectory();
    gettimeofday(&tend, NULL);
    xList = lastTraj.xList;
    uList = lastTraj.uList;
    iter = lastTraj.iter;

    xinit << xList[1](0, 0), xList[1](1, 0);
    xDes << vec_joint_pos[i + 1], 0.0;

    texec += ((double)(tend.tv_sec - tbegin.tv_sec) * 1000.0 +
              ((double)(tend.tv_usec - tbegin.tv_usec) / 1000.0));
    cout << "texec:" << texec << std::endl;

    string ResultNumber;
    ostringstream convert;
    convert << i;
    ResultNumber = convert.str();

    ofstream file(("results_sinu/results_sinu" + ResultNumber + ".csv").c_str(),
                  ios::out | ios::trunc);
    if (file) {
      file << "q,qdes,qDot,qDotDes,u" << endl;
      for (unsigned int i = 0; i < T; i++) {
        file << xList[i](0, 0) << "," << xDes[0] << "," << xList[i](1, 0) << ","
             << xDes[1] << "," << uList[i - 1](0, 0) << endl;
      }
      file << xList[T](0, 0) << "," << xDes[0] << "," << xList[T](1, 0) << ","
           << xDes[1] << "," << uList[T - 1](0, 0) << endl;
      file.close();
    } else {
      cerr << "erreur ouverte fichier" << endl;
    }

    if (i == 0) {
      ofstream fichier(("results_sinu/results_sinu.csv"),
                       ios::out | ios::trunc);
      if (fichier) {
        fichier << "q,qdes,qDot,qDotDes,u" << endl;
      }
    }

    ofstream fichier(("results_sinu/results_sinu.csv"), ios::out | ios::app);
    if (fichier) {
      fichier << xList[1](0, 0) << "," << xDes[0] << "," << xList[1](1, 0)
              << "," << xDes[1] << "," << uList[0](0, 0) << endl;
      fichier.close();
    } else {
      cerr << "erreur ouverte fichier" << endl;
    }
  }
  cout << endl;
  cout << "temps d'execution total du solveur ";
  cout << texec << endl;
  cout << "temps d'execution par pas de temps ";
  cout << texec / (double)nbIterTestMax << endl;
  cout << "Nombre d'itérations : " << iter << endl;

  return 0;
}
