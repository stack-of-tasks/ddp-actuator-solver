#ifndef LQRSOLVER_H
#define LQRSOLVER_H

#include "dynamicmodel.h"
#include "costfunction.h"

#include <Eigen/Dense>
#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::MatrixXd)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXd)

template<typename precision,int stateSize,int commandSize>
class LQRSolver
{
public:
    typedef Eigen::Matrix<precision,stateSize,1> stateVec_t;                  // 1 x stateSize
    typedef Eigen::Matrix<precision,1,stateSize> stateVecTrans_t;                  // 1 x stateSize
    typedef Eigen::Matrix<precision,stateSize,stateSize> stateMat_t;               // stateSize x stateSize
    typedef Eigen::Matrix<precision,stateSize,stateSize> stateTens_t[stateSize];   // stateSize x stateSize x stateSize

    // typedef for commandSize types
    typedef Eigen::Matrix<precision,commandSize,1> commandVec_t;                           // commandSize x 1
    typedef Eigen::Matrix<precision,1,commandSize> commandVecTrans_t;                      // 1 x commandSize
    typedef Eigen::Matrix<precision,commandSize,commandSize> commandMat_t;                 // commandSize x commandSize
    typedef Eigen::Matrix<precision,commandSize,commandSize> commandTens_t[commandSize];   // stateSize x commandSize x commandSize



    // typedef for mixed stateSize and commandSize types
    typedef Eigen::Matrix<precision,stateSize,commandSize> stateR_commandC_t;                          // stateSize x commandSize
    typedef Eigen::Matrix<precision,stateSize,commandSize> stateR_commandC_stateD_t[stateSize];        // stateSize x commandSize x stateSize
    typedef Eigen::Matrix<precision,stateSize,commandSize> stateR_commandC_commandD_t[commandSize];    // stateSize x commandSize x commandSize
    typedef Eigen::Matrix<precision,commandSize,stateSize> commandR_stateC_t;                          // commandSize x stateSize
    typedef Eigen::Matrix<precision,commandSize,stateSize> commandR_stateC_stateD_t[stateSize];        // commandSize x stateSize x stateSize
    typedef Eigen::Matrix<precision,commandSize,stateSize> commandR_stateC_commandD_t[commandSize];    // commandSize x stateSize x commandSize
    typedef Eigen::Matrix<precision,stateSize,stateSize> stateR_stateC_commandD_t[commandSize];    // stateSize x stateSize x commandSize
    typedef Eigen::Matrix<precision,commandSize,commandSize> commandR_commandC_stateD_t[stateSize];    // commandSize x commandSize x stateSize

    typedef std::vector<stateVec_t> stateVecTab_t;
    typedef std::vector<commandVec_t> commandVecTab_t;
    typedef std::vector<stateMat_t> stateMatTab_t;
    typedef std::vector<commandR_stateC_t> commandR_stateC_tab_t;

    typedef DynamicModel<precision,stateSize,commandSize> DynamicModel_t;
    typedef CostFunction<precision,stateSize,commandSize> CostFunction_t;

public:
    struct traj
    {
        stateVecTab_t xList;
        commandVecTab_t uList;
    };

public:
private:
protected:
    // attributes //
public:
private:
    DynamicModel_t * dynamicModel;
    CostFunction_t * costFunction;
    unsigned int stateNb;
    unsigned int commandNb;
    stateVec_t x;
    commandVec_t u;
    stateVec_t xInit;
    stateVec_t xDes;
    unsigned int T;
    double dt;

    stateMat_t Q;
    commandMat_t R;

    stateMat_t A;
    stateR_commandC_t B;

    stateMat_t P;
    commandR_stateC_t F;

    stateMatTab_t PList;
    commandR_stateC_tab_t FList;

    stateVecTab_t xList;
    commandVecTab_t uList;
    struct traj lastTraj;

protected:
    // methods //
public:
    LQRSolver(DynamicModel_t& myDynamicModel, CostFunction_t& myCostFunction)
    {
        dynamicModel = &myDynamicModel;
        costFunction = &myCostFunction;
        stateNb = myDynamicModel.getStateNb();
        commandNb = myDynamicModel.getCommandNb();
        Q = myCostFunction.getlxx();
        R = myCostFunction.getluu();
        A = myDynamicModel.getfx();
        B = myDynamicModel.getfu();
    }

    void InitSolver(stateVec_t& myxInit, stateVec_t& myxDes, unsigned int& myT, double& mydt)
    {
        xInit = myxInit-myxDes;
        xDes = myxDes;
        T = myT;
        dt = mydt;

        xList.resize(myT+1);
        uList.resize(myT);

        PList.resize(myT+1);
        FList.resize(myT);
    }

    void solveTrajectory()
    {
        backwardPhase();
        forwardPhase();
    }

    void backwardPhase()
    {
        PList[T] = Q;
        for(int i=T-1;i>0;i--) PList[i] = A.transpose()*PList[i+1]*A - (A.transpose()*PList[i+1]*B)*((R+B.transpose()*PList[i+1]*B).inverse())*(B.transpose()*PList[i+1]*A)+Q;
        for(int i=0;i<T;i++) FList[i] = ((R+B.transpose()*PList[i]*B).inverse())*(B.transpose()*PList[i+1]*A);
    }

    void forwardPhase()
    {
        xList[0] = xInit;
        for(int i=0;i<T;i++)
        {
            uList[i] = - FList[i]*(xList[i]);
            xList[i+1] = A*xList[i] + B*uList[i];
        }
    }

    LQRSolver::traj getLastSolvedTrajectory()
    {
        lastTraj.xList = xList;
        for(int i=0;i<T+1;i++) lastTraj.xList[i] = lastTraj.xList[i] + xDes;
        lastTraj.uList = uList;
        return lastTraj;
    }
};

#endif // LQRSOLVER_H
