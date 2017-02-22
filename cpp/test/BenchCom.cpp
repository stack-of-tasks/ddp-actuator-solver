#include "BenchCom.h"

BenchCom::BenchCom()
{
    ser = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );
    struct termios tty;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( ser, &tty ) != 0 ) {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

    /* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B1000000);
    cfsetispeed (&tty, (speed_t)B1000000);

    /* Setting other Port Stuff */
    /*config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                     INLCR | PARMRK | INPCK | ISTRIP | IXON);
    config.c_oflag = 0;*/
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;

    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  1;                  // read doesn't block
    tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush( ser, TCIFLUSH );
    if ( tcsetattr ( ser, TCSANOW, &tty ) != 0) {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }

    current_offset = 2048.0;

    full_state = getOffset();
    current_offset = full_state.current;
    joint_offset = full_state.jointPos;
    motor_offset = full_state.motorPos;
}

uint8_t BenchCom::crc8(uint8_t crc, uint8_t crc_data)
{
    uint8_t i;
    i = (crc_data ^ crc) & 0xff;
    crc = 0;
    if(i & 1)
        crc ^= 0x5e;
    if(i & 2)
        crc ^= 0xbc;
    if(i & 4)
        crc ^= 0x61;
    if(i & 8)
        crc ^= 0xc2;
    if(i & 0x10)
        crc ^= 0x9d;
    if(i & 0x20)
        crc ^= 0x23;
    if(i & 0x40)
        crc ^= 0x46;
    if(i & 0x80)
        crc ^= 0x8c;
    return(crc);
}

struct BenchCom::fullState BenchCom::getOffset()
{
    crc = 0;
    send = 0x47;
    write(ser,&send,1);
    send = 0x01;
    crc = crc8(crc,send);
    write(ser,&send,1);
    send = crc;
    write(ser,&send,1);

    do
    {
        read(ser,&received,1);
    }while(received!=0x47);

    read(ser,&motorH,1);
    read(ser,&motorL,1);
    read(ser,&motorTurn,1);
    read(ser,&jointH,1);
    read(ser,&jointL,1);
    read(ser,&currentH,1);
    read(ser,&currentL,1);
    read(ser,&crc1,1);

    crcCalc1 = 0;
    crcCalc1 = crc8(crcCalc1,motorH);
    crcCalc1 = crc8(crcCalc1,motorL);
    crcCalc1 = crc8(crcCalc1,motorTurn);
    crcCalc1 = crc8(crcCalc1,jointH);
    crcCalc1 = crc8(crcCalc1,jointL);
    crcCalc1 = crc8(crcCalc1,currentH);
    crcCalc1 = crc8(crcCalc1,currentL);

    motorPosOld = motorPos;
    jointPosOld = jointPos;

    motorPosInt = (4096*motorTurn)+(motorH<<8)|motorL;
    jointPosInt = (jointH<<8)|jointL;
    currentInt = (currentH<<8)|currentL;
    motorPos = (double)motorPosInt*M_PI/2048.0;
    jointPos = (double)jointPosInt*M_PI/2048.0;
    current = (double) ( ( currentInt )*(3.0/(4096.0*20.0*0.005)));

    full_state.jointPos = jointPos;
    full_state.jointVel = 0.0;
    full_state.motorPos = motorPos;
    full_state.motorVel = 0.0;
    full_state.current = current;

    full_state.jointPosInt = jointPosInt;
    full_state.motorPosInt = motorPosInt;
    full_state.currentInt = currentInt;

    return full_state;
}

struct BenchCom::fullState BenchCom::getStateFromSerial() {
    crc = 0;
    send = 0x47;
    write(ser, &send, 1);
    send = 0x06;
    crc = crc8(crc, send);
    write(ser, &send, 1);

    posTarget = 1 << 15;

    send = (posTarget >> 8) & 0xFF;
    crc = crc8(crc, send);
    write(ser, &send, 1);
    send = posTarget & 0xFF;
    crc = crc8(crc, send);
    write(ser, &send, 1);
    send = crc;
    write(ser, &send, 1);

    last_time = current_time;
    gettimeofday(&current_time, NULL);
    diff_time = ((current_time.tv_sec - last_time.tv_sec) + (current_time.tv_usec - last_time.tv_usec) / 1000000.0);

    do {
        read(ser, &received, 1);
    } while (received != 0x47);

    read(ser, &motorH, 1);
    read(ser, &motorL, 1);
    read(ser, &motorTurn, 1);
    read(ser, &jointH, 1);
    read(ser, &jointL, 1);
    read(ser, &currentH, 1);
    read(ser, &currentL, 1);
    read(ser, &pwmH, 1);
    read(ser, &pwmL, 1);
    read(ser, &crc1, 1);
    read(ser, &crc2, 1);

    crcCalc1 = 0;
    crcCalc1 = crc8(crcCalc1, motorH);
    crcCalc1 = crc8(crcCalc1, motorL);
    crcCalc1 = crc8(crcCalc1, motorTurn);

    crcCalc2 = 0;
    crcCalc2 = crc8(crcCalc2, jointH);
    crcCalc2 = crc8(crcCalc2, jointL);
    crcCalc2 = crc8(crcCalc2, currentH);
    crcCalc2 = crc8(crcCalc2, currentL);

    motorPosOld = motorPos;
    jointPosOld = jointPos;

    motorPosInt = (4096 * motorTurn) + (motorH << 8) | motorL;
    jointPosInt = (jointH << 8) | jointL;
    currentInt = (currentH << 8) | currentL;
    motorPos = (double) motorPosInt * M_PI / 2048.0;
    jointPos = (double) jointPosInt * M_PI / 2048.0;
    motorVel = (motorPos - motorPosOld) / diff_time;
    jointVel = (jointPos - jointPosOld) / diff_time;
    current = (double) ((currentInt - current_offset) * (3.0 / (4096.0 * 20.0 * 0.005)));

    full_state.jointPos = jointPos;
    full_state.jointVel = jointVel;
    full_state.motorPos = motorPos;
    full_state.motorVel = motorVel;
    full_state.current = current;

    full_state.jointPosInt = jointPosInt;
    full_state.motorPosInt = motorPosInt;
    full_state.currentInt = currentInt;

    return full_state;
}

struct BenchCom::fullState BenchCom::sendCurrentCommand(double current, double jointDes)
{
    currentInt = (int16_t) ((current)*((4096.0*20.0*0.005)/3.0));
    jointInt = (uint16_t) (jointDes*(2048.0/M_PI));

    std::cout << jointInt << std::endl;

    crc = 0;
    send = 0x47;
    write(ser,&send,1);
    send = 0x03;
    crc = crc8(crc,send);
    write(ser,&send,1);

    send = (jointInt >> 8)&0xFF;
    crc = crc8(crc,send);
    write(ser,&send,1);
    send = jointInt & 0xFF;
    crc = crc8(crc,send);
    write(ser,&send,1);

    send = (currentInt >> 8)&0xFF;
    crc = crc8(crc,send);
    write(ser,&send,1);
    send = currentInt & 0xFF;
    crc = crc8(crc,send);
    write(ser,&send,1);
    send = crc;
    write(ser,&send,1);

    last_time = current_time;
    gettimeofday(&current_time,NULL);
    diff_time = ((current_time.tv_sec-last_time.tv_sec) + (current_time.tv_usec-last_time.tv_usec)/1000000.0);

    do
    {
        read(ser,&received,1);
    }while(received!=0x47);

    read(ser,&motorH,1);
    read(ser,&motorL,1);
    read(ser,&motorTurn,1);
    read(ser,&jointH,1);
    read(ser,&jointL,1);
    read(ser,&currentH,1);
    read(ser,&currentL,1);
    read(ser,&pwmH,1);
    read(ser,&pwmL,1);
    read(ser,&crc1,1);
    read(ser,&crc2, 1);

    crcCalc1 = 0;
    crcCalc1 = crc8(crcCalc1, motorH);
    crcCalc1 = crc8(crcCalc1, motorL);
    crcCalc1 = crc8(crcCalc1, motorTurn);

    crcCalc2 = 0;
    //crcCalc2 = crc8(crcCalc2, jointH);
    //crcCalc2 = crc8(crcCalc2, jointL);
    crcCalc2 = crc8(crcCalc2, currentH);
    //crcCalc2 = crc8(crcCalc2, currentL);

    motorPosOld = motorPos;
    jointPosOld = jointPos;

    motorPosInt = (4096*motorTurn)+(motorH<<8)|motorL;
    jointPosInt = (jointH<<8)|jointL;
    currentInt = (currentH<<8)|currentL;
    motorPos = (double) motorPosInt*M_PI/2048.0;
    jointPos = (double) jointPosInt*M_PI/2048.0;
    motorVel = (motorPos - motorPosOld)/diff_time;
    jointVel = (jointPos - jointPosOld)/diff_time;
    current = (double) ( ( currentInt - current_offset )*(3.0/(4096.0*20.0*0.005)));

    full_state.jointPos = jointPos;
    full_state.jointVel = jointVel;
    full_state.motorPos = motorPos;
    full_state.motorVel = motorVel;
    full_state.current = current;

    full_state.jointPosInt = jointPosInt;
    full_state.motorPosInt = motorPosInt;
    full_state.currentInt = currentInt;

    return full_state;
}

struct BenchCom::fullState BenchCom::sendPositionCommand(double jointDes)
{
    jointInt = (uint16_t) (jointDes*(2048.0/M_PI));

    crc = 0;
    send = 0x47;
    write(ser,&send,1);
    send = 0x02;
    crc = crc8(crc,send);
    write(ser,&send,1);

    send = (jointInt >> 8)&0xFF;
    crc = crc8(crc,send);
    write(ser,&send,1);
    send = jointInt & 0xFF;
    crc = crc8(crc,send);
    write(ser,&send,1);
    send = crc;
    write(ser,&send,1);

    last_time = current_time;
    gettimeofday(&current_time,NULL);
    diff_time = ((current_time.tv_sec-last_time.tv_sec) + (current_time.tv_usec-last_time.tv_usec)/1000000.0);

    do
    {
        read(ser,&received,1);
    }while(received!=0x47);

    read(ser,&motorH,1);
    read(ser,&motorL,1);
    read(ser,&motorTurn,1);
    read(ser,&jointH,1);
    read(ser,&jointL,1);
    read(ser,&currentH,1);
    read(ser,&currentL,1);
    read(ser,&pwmH,1);
    read(ser,&pwmL,1);
    read(ser,&crc1,1);
    read(ser, &crc2, 1);

    crcCalc1 = 0;
    crcCalc1 = crc8(crcCalc1, motorH);
    crcCalc1 = crc8(crcCalc1, motorL);
    crcCalc1 = crc8(crcCalc1, motorTurn);

    crcCalc2 = 0;
    crcCalc2 = crc8(crcCalc2, jointH);
    crcCalc2 = crc8(crcCalc2, jointL);
    crcCalc2 = crc8(crcCalc2, currentH);
    crcCalc2 = crc8(crcCalc2, currentL);

    motorPosOld = motorPos;
    jointPosOld = jointPos;

    motorPosInt = (4096*motorTurn)+(motorH<<8)|motorL;
    jointPosInt = (jointH<<8)|jointL;
    currentInt = (currentH<<8)|currentL;
    motorPos = (double) motorPosInt*M_PI/2048.0;
    jointPos = (double) jointPosInt*M_PI/2048.0;
    motorVel = (motorPos - motorPosOld)/diff_time;
    jointVel = (jointPos - jointPosOld)/diff_time;
    current = (double) ( ( currentInt - current_offset )*(3.0/(4096.0*20.0*0.005)));

    full_state.jointPos = jointPos;
    full_state.jointVel = jointVel;
    full_state.motorPos = motorPos;
    full_state.motorVel = motorVel;
    full_state.current = current;

    full_state.jointPosInt = jointPosInt;
    full_state.motorPosInt = motorPosInt;
    full_state.currentInt = currentInt;

    return full_state;
}

struct BenchCom::fullState BenchCom::sendStopCommand()
{
    crc = 0;
    send = 0x47;
    write(ser,&send,1);
    send = 0x00;
    crc = crc8(crc,send);
    write(ser,&send,1);
    send = crc;
    write(ser,&send,1);

    last_time = current_time;
    gettimeofday(&current_time,NULL);
    diff_time = ((current_time.tv_sec-last_time.tv_sec) + (current_time.tv_usec-last_time.tv_usec)/1000000.0);

    do
    {
        read(ser,&received,1);
    }while(received!=0x47);

    read(ser,&motorH,1);
    read(ser,&motorL,1);
    read(ser,&motorTurn,1);
    read(ser,&jointH,1);
    read(ser,&jointL,1);
    read(ser,&currentH,1);
    read(ser,&currentL,1);
    read(ser,&pwmH,1);
    read(ser,&pwmL,1);
    read(ser,&crc1,1);
    read(ser, &crc2, 1);

    crcCalc1 = 0;
    crcCalc1 = crc8(crcCalc1, motorH);
    crcCalc1 = crc8(crcCalc1, motorL);
    crcCalc1 = crc8(crcCalc1, motorTurn);

    crcCalc2 = 0;
    crcCalc2 = crc8(crcCalc2, jointH);
    crcCalc2 = crc8(crcCalc2, jointL);
    crcCalc2 = crc8(crcCalc2, currentH);
    crcCalc2 = crc8(crcCalc2, currentL);

    motorPosOld = motorPos;
    jointPosOld = jointPos;

    motorPosInt = (4096*motorTurn)+(motorH<<8)|motorL;
    jointPosInt = (jointH<<8)|jointL;
    currentInt = (currentH<<8)|currentL;
    motorPos = (double) motorPosInt*M_PI/2048.0;
    jointPos = (double) jointPosInt*M_PI/2048.0;
    motorVel = (motorPos - motorPosOld)/diff_time;
    jointVel = (jointPos - jointPosOld)/diff_time;
    current = (double) ( ( currentInt - current_offset )*(3.0/(4096.0*20.0*0.005)));

    full_state.jointPos = jointPos;
    full_state.jointVel = jointVel;
    full_state.motorPos = motorPos;
    full_state.motorVel = motorVel;
    full_state.current = current;

    full_state.jointPosInt = jointPosInt;
    full_state.motorPosInt = motorPosInt;
    full_state.currentInt = currentInt;

    return full_state;
}

struct BenchCom::fullState BenchCom::SimulatesendCurrentCommand(struct fullState current_state, double current, double jointDes)
{
    double k = 588.0;
    double R = 96.1;
    double Jm = 183 * 1e-7;
    double Jl = 0.000085;
    double fvm = 5.65e-5;
    double fvl = 0.278;
    double Kt = 0.0578;
    double mu = 0.52;
    double Cf0 = 0.0;
    double a = 0.0;

    double dt = 0.001;

    x << current_state.jointPos,current_state.jointVel,current_state.motorPos,current_state.motorVel;

    A <<   0.0,1.0,0.0,0.0,
            -k/Jl,-fvl/Jl,k/(R*Jl),0.0,
            0.0,0.0,0.0,1.0,
            k/(R*Jm),0.0,-k/(Jm*R*R),-fvm/Jm;

    Ad = (dt*A).exp();

    B << 0.0,0.0,0.0,Kt/Jm;
    Bd = dt*B;

    x_next = Ad*x - Bd*current;

    full_state.jointPos = x_next(0,0);
    full_state.jointVel = x_next(1,0);
    full_state.motorPos = x_next(2,0);
    full_state.motorVel = x_next(3,0);

    return full_state;
}

BenchCom::stateVec_t BenchCom::getStateFromSerialWithoutHal() {
    crc = 0;
    send = 0x47;
    write(ser, &send, 1);
    send = 0x06;
    crc = crc8(crc, send);
    write(ser, &send, 1);

    posTarget = 1 << 15;

    send = (posTarget >> 8) & 0xFF;
    crc = crc8(crc, send);
    write(ser, &send, 1);
    send = posTarget & 0xFF;
    crc = crc8(crc, send);
    write(ser, &send, 1);
    send = crc;
    write(ser, &send, 1);

    last_time = current_time;
    gettimeofday(&current_time, NULL);
    diff_time = ((current_time.tv_sec - last_time.tv_sec) + (current_time.tv_usec - last_time.tv_usec) / 1000000.0);

    do {
        read(ser, &received, 1);
    } while (received != 0x47);

    read(ser, &motorH, 1);
    read(ser, &motorL, 1);
    read(ser, &motorTurn, 1);
    read(ser, &jointH, 1);
    read(ser, &jointL, 1);
    read(ser, &currentH, 1);
    read(ser, &currentL, 1);
    read(ser, &pwmH, 1);
    read(ser, &pwmL, 1);
    read(ser, &crc1, 1);
    read(ser, &crc2, 1);

    crcCalc1 = 0;
    crcCalc1 = crc8(crcCalc1, motorH);
    crcCalc1 = crc8(crcCalc1, motorL);
    crcCalc1 = crc8(crcCalc1, motorTurn);

    crcCalc2 = 0;
    crcCalc2 = crc8(crcCalc2, jointH);
    crcCalc2 = crc8(crcCalc2, jointL);
    crcCalc2 = crc8(crcCalc2, currentH);
    crcCalc2 = crc8(crcCalc2, currentL);

    motorPosIntOld = motorPosInt;
    jointPosIntOld = jointPosInt;

    motorPosInt = (4096*motorTurn)+(motorH<<8)|motorL;
    jointPosInt = (jointH<<8)|jointL;
    currentInt = (currentH<<8)|currentL;
    motorVelInt = (motorPosInt - motorPosIntOld)/diff_time;
    jointVelInt = (jointPosInt - jointPosIntOld)/diff_time;

    x << jointPosInt,jointVelInt,motorPosInt,motorVelInt;
    return x;
}

BenchCom::stateVec_t BenchCom::getStateFromSerialWithoutHalIdle() {
    crc = 0;
    send = 0x47;
    write(ser, &send, 1);
    send = 0x00;
    crc = crc8(crc, send);
    write(ser, &send, 1);
    send = crc;
    write(ser, &send, 1);

    last_time = current_time;
    gettimeofday(&current_time, NULL);
    diff_time = ((current_time.tv_sec - last_time.tv_sec) + (current_time.tv_usec - last_time.tv_usec) / 1000000.0);

    do {
        read(ser, &received, 1);
    } while (received != 0x47);

    read(ser, &motorH, 1);
    read(ser, &motorL, 1);
    read(ser, &motorTurn, 1);
    read(ser, &jointH, 1);
    read(ser, &jointL, 1);
    read(ser, &currentH, 1);
    read(ser, &currentL, 1);
    read(ser, &pwmH, 1);
    read(ser, &pwmL, 1);
    read(ser, &crc1, 1);
    read(ser, &crc2, 1);

    crcCalc1 = 0;
    crcCalc1 = crc8(crcCalc1, motorH);
    crcCalc1 = crc8(crcCalc1, motorL);
    crcCalc1 = crc8(crcCalc1, motorTurn);

    crcCalc2 = 0;
    crcCalc2 = crc8(crcCalc2, jointH);
    crcCalc2 = crc8(crcCalc2, jointL);
    crcCalc2 = crc8(crcCalc2, currentH);
    crcCalc2 = crc8(crcCalc2, currentL);

    motorPosIntOld = motorPosInt;
    jointPosIntOld = jointPosInt;

    motorPosInt = (4096*motorTurn)+(motorH<<8)|motorL;
    jointPosInt = (jointH<<8)|jointL;
    currentInt = (currentH<<8)|currentL;
    motorVelInt = (motorPosInt - motorPosIntOld)/diff_time;
    jointVelInt = (jointPosInt - jointPosIntOld)/diff_time;

    x << jointPosInt,jointVelInt,motorPosInt,motorVelInt;
    return x;
}

BenchCom::stateVec_t BenchCom::sendCurrentCommandWithoutHal(int16_t current, uint16_t jointDes)
{
    crc = 0;
    send = 0x47;
    write(ser,&send,1);
    send = 0x03;
    crc = crc8(crc,send);
    write(ser,&send,1);

    send = (jointDes >> 8)&0xFF;
    crc = crc8(crc,send);
    write(ser,&send,1);
    send = jointDes & 0xFF;
    crc = crc8(crc,send);
    write(ser,&send,1);

    send = (current >> 8)&0xFF;
    crc = crc8(crc,send);
    write(ser,&send,1);
    send = current & 0xFF;
    crc = crc8(crc,send);
    write(ser,&send,1);
    send = crc;
    write(ser,&send,1);

    last_time = current_time;
    gettimeofday(&current_time,NULL);
    diff_time = ((current_time.tv_sec-last_time.tv_sec) + (current_time.tv_usec-last_time.tv_usec)/1000000.0);

    do
    {
        read(ser,&received,1);
    }while(received!=0x47);

    read(ser,&motorH,1);
    read(ser,&motorL,1);
    read(ser,&motorTurn,1);
    read(ser,&jointH,1);
    read(ser,&jointL,1);
    read(ser,&currentH,1);
    read(ser,&currentL,1);
    read(ser,&pwmH,1);
    read(ser,&pwmL,1);
    read(ser,&crc1,1);
    read(ser, &crc2, 1);

    crcCalc1 = 0;
    crcCalc1 = crc8(crcCalc1, motorH);
    crcCalc1 = crc8(crcCalc1, motorL);
    crcCalc1 = crc8(crcCalc1, motorTurn);

    crcCalc2 = 0;
    crcCalc2 = crc8(crcCalc2, jointH);
    crcCalc2 = crc8(crcCalc2, jointL);
    crcCalc2 = crc8(crcCalc2, currentH);
    crcCalc2 = crc8(crcCalc2, currentL);

    motorPosIntOld = motorPosInt;
    jointPosIntOld = jointPosInt;

    motorPosInt = (4096*motorTurn)+(motorH<<8)|motorL;
    jointPosInt = (jointH<<8)|jointL;
    currentInt = (currentH<<8)|currentL;
    motorVelInt = (motorPosInt - motorPosIntOld)/diff_time;
    jointVelInt = (jointPosInt - jointPosIntOld)/diff_time;

    //x << jointPosInt,jointVelInt,motorPosInt,motorVelInt;
    x << jointPosInt,0.0,motorPosInt,0.0;
    return x;
}

BenchCom::stateVec_t BenchCom::sendPositionCommandWithoutHal(uint16_t jointDes)
{
    crc = 0;
    send = 0x47;
    write(ser,&send,1);
    send = 0x02;
    crc = crc8(crc,send);
    write(ser,&send,1);

    send = (jointDes >> 8)&0xFF;
    crc = crc8(crc,send);
    write(ser,&send,1);
    send = jointDes & 0xFF;
    crc = crc8(crc,send);
    write(ser,&send,1);
    send = crc;
    write(ser,&send,1);

    last_time = current_time;
    gettimeofday(&current_time,NULL);
    diff_time = ((current_time.tv_sec-last_time.tv_sec) + (current_time.tv_usec-last_time.tv_usec)/1000000.0);

    do
    {
        read(ser,&received,1);
    }while(received!=0x47);

    read(ser,&motorH,1);
    read(ser,&motorL,1);
    read(ser,&motorTurn,1);
    read(ser,&jointH,1);
    read(ser,&jointL,1);
    read(ser,&currentH,1);
    read(ser,&currentL,1);
    read(ser,&pwmH,1);
    read(ser,&pwmL,1);
    read(ser,&crc1,1);
    read(ser, &crc2, 1);

    crcCalc1 = 0;
    crcCalc1 = crc8(crcCalc1, motorH);
    crcCalc1 = crc8(crcCalc1, motorL);
    crcCalc1 = crc8(crcCalc1, motorTurn);

    crcCalc2 = 0;
    crcCalc2 = crc8(crcCalc2, jointH);
    crcCalc2 = crc8(crcCalc2, jointL);
    crcCalc2 = crc8(crcCalc2, currentH);
    crcCalc2 = crc8(crcCalc2, currentL);

    motorPosIntOld = motorPosInt;
    jointPosIntOld = jointPosInt;

    printf("%i\n", motorTurn );

    motorPosInt = (4096*motorTurn)+(motorH<<8)|motorL;
    jointPosInt = (jointH<<8)|jointL;
    currentInt = (currentH<<8)|currentL;
    motorVelInt = (motorPosInt - motorPosIntOld)/diff_time;
    jointVelInt = (jointPosInt - jointPosIntOld)/diff_time;

    x << jointPosInt,jointVelInt,motorPosInt,motorVelInt;

    return x;
}