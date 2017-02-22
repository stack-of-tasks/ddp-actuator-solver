#ifndef PROJECT_BENCHCOM_H
#define PROJECT_BENCHCOM_H

#include <iostream>
#include <fstream>
#include <cerrno>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include <bitset>

#include <math.h>

#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

#include <Eigen/Core>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

class BenchCom
{
public:
    BenchCom();

public:
    typedef Eigen::Matrix<double,4,1> stateVec_t;
    typedef Eigen::Matrix<double,4,4> stateMat_t;               // stateSize x stateSize
    typedef Eigen::Matrix<double,4,1> stateR_commandC_t;                          // stateSize x commandSize

    struct fullState
    {
        double jointPos;
        double jointVel;
        double motorPos;
        double motorVel;
        double current;
        uint16_t jointPosInt;
        int32_t motorPosInt;
        uint32_t currentInt;
    };
private:
    int ser;
    struct fullState full_state;
    struct timeval current_time,last_time;
    double diff_time;
    uint8_t crc,crc1,crc2,crcCalc1,crcCalc2;
    uint8_t send;
    uint8_t received;
    uint8_t motorH,motorL;
    uint8_t jointH,jointL;
    uint8_t pwmH,pwmL;
    int8_t motorTurn;
    uint8_t currentH,currentL;
    int32_t motorPosInt;
    int32_t jointVelInt;
    int32_t motorVelInt;
    int32_t motorPosIntOld;
    uint32_t jointPosIntOld;
    uint32_t jointPosInt;
    uint16_t jointInt;
    int32_t currentInt;
    double current_offset;
    double joint_offset;
    double motor_offset;
    double jointPos;
    double motorPos;
    double jointVel;
    double motorVel;
    double jointPosOld;
    double motorPosOld;
    double current;
    uint16_t posTarget;

    stateVec_t x;
    stateVec_t x_next;
    stateMat_t A;
    stateMat_t Ad;
    stateR_commandC_t B;
    stateR_commandC_t Bd;

private:
    struct fullState getOffset();
public:
    uint8_t crc8(uint8_t crc, uint8_t crc_data);
    struct fullState getStateFromSerial();

    struct fullState sendCurrentCommand(double current, double jointDes);
    struct fullState sendPositionCommand(double jointDes);
    struct fullState sendStopCommand();

    stateVec_t getStateFromSerialWithoutHal();
    stateVec_t getStateFromSerialWithoutHalIdle();
    stateVec_t sendCurrentCommandWithoutHal(int16_t current, uint16_t jointDes);
    stateVec_t sendPositionCommandWithoutHal(uint16_t jointDes);

    struct fullState SimulatesendCurrentCommand(struct fullState current_state, double current, double jointDes);

// accessor
    double getCurrentOffset(){return current_offset;}
    double getJointOffset(){return joint_offset;}
    double getMotorOffset(){return motor_offset;}

    uint16_t getCurrentOffsetInt(){return (uint16_t) (current_offset*((4096.0*20.0*0.005)/3.0));}
    uint16_t getJointOffsetInt(){return (uint16_t) (joint_offset*(2048.0/M_PI));}
    int16_t getMotorOffsetInt(){return (int16_t) (motor_offset*(2048.0/M_PI));}

    double CurrentFWToSI(uint16_t currentInt){return (double)((currentInt-current_offset)*(3.0/(4096.0*20.0*0.005)));}
    int16_t CurrentSIToFW(double currentSI){return (int16_t)(currentSI*((4096.0*20.0*0.005)/3.0));}
    double AngleFWToSI(int16_t angleInt){return (double) (angleInt*(M_PI/2048.0));}
    int16_t AngleSIToFW(double angleSI){return (int16_t) (angleSI*(2048.0/M_PI));}
};


#endif //PROJECT_BENCHCOM_H
