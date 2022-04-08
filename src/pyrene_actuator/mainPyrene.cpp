#include <sys/time.h>
#include <time.h>

#include <ddp-actuator-solver/ddpsolver.hh>
#include <fstream>
#include <iostream>

#include "ddp-actuator-solver/pyrene_actuator/pyreneActuator.hh"
#include "ddp-actuator-solver/pyrene_actuator/pyreneCostFunction.hh"

using namespace std;
using namespace Eigen;

int main() {
  struct timeval tbegin, tend;
  double texec = 0.0;

  DDPSolver<double, 2, 1>::stateVec_t xinit, xDes;

  xinit << -1.29653, 0.973714;
  xDes << -1.3, 0;

  unsigned int T = 50;
  double dt = 1e-3;
  unsigned int iterMax = 100;
  double stopCrit = 1e-3;

  DDPSolver<double, 2, 1>::stateVecTab_t xList;
  DDPSolver<double, 2, 1>::commandVecTab_t uList;
  DDPSolver<double, 2, 1>::traj lastTraj;

  pyreneActuator pyreneActuator;
  CostFunctionPyreneActuator costFunction;
  costFunction.setTauLimit(70);
  costFunction.setJointLimit(0.0, -2.35619449019);
  costFunction.setJointVelLimit(30.0, -30.0);
  DDPSolver<double, 2, 1> testSolverActuator(pyreneActuator, costFunction,
                                             DISABLE_FULLDDP, DISABLE_QPBOX);
  testSolverActuator.FirstInitSolver(xinit, xDes, T, dt, iterMax, stopCrit);

  int N = 100;
  gettimeofday(&tbegin, NULL);
  for (int i = 0; i < N; i++) testSolverActuator.solveTrajectory();
  gettimeofday(&tend, NULL);

  lastTraj = testSolverActuator.getLastSolvedTrajectory();
  xList = lastTraj.xList;
  uList = lastTraj.uList;
  unsigned int iter = lastTraj.iter;
  texec = ((double)(1000 * (tend.tv_sec - tbegin.tv_sec) +
                    ((tend.tv_usec - tbegin.tv_usec) / 1000))) /
          1000.;
  texec /= N;

  cout << endl;
  cout << "temps d'execution total du solveur ";
  cout << texec << endl;
  cout << "temps d'execution par pas de temps ";
  cout << texec / T << endl;
  cout << "Nombre d'itérations : " << iter << endl;

  ofstream fichier("results_simple.csv", ios::out | ios::trunc);
  if (fichier) {
    fichier << "q,qDot,u" << endl;
    for (unsigned int i = 0; i < T; i++) {
      fichier << xList[i](0, 0) << "," << xList[i](1, 0) << ","
              << uList[i](0, 0) << endl;
    }
    fichier << xList[T](0, 0) << "," << xList[T](1, 0) << ","
            << uList[T - 1](0, 0) << endl;
    fichier.close();

  } else
    cerr << "erreur ouverte fichier" << endl;
  return 0;
}
