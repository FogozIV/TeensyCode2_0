//
// Created by fogoz on 11/02/2025.
//

#ifndef TEENSYCODE2_0_RAMP_H
#define TEENSYCODE2_0_RAMP_H

#include "../robot/AbstractRobot.h"
#include "chrono"

#define SIGN(x) (x < 0 ? -1 : 1)

class Ramp{
protected:
    double maxAcc;
    double maxDec;
    double maxSpeed;

    double sign;

    bool increment = false;

    struct{
        double initialSpeed;
        double currentSpeed;
        double endSpeed;
        double accDistance;
        double decDistance;

        double pointDistance;
        double wheelDistance;
    } signCorrected;

    struct Private{
        bool done;
        double speed;
        double distance;
    };

    struct RampReturnData{
        bool end;
        double distance_increment;
        double speed;
        bool stop;
        bool started;
    };

    std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> initTime;

    Ramp::Private previousComputation = {false, NAN, NAN};

    double accTime;
    double decTime;
    double steTime;

    double initSteTime;

    double steSpeed;

public:
    Ramp(double acc, double dec, double maxSpeed) : maxAcc(acc), maxDec(dec), maxSpeed(maxSpeed){

    }

    void setTarget(const AbstractRobot& robot, double distance, double initialSpeed=0.0f, double endSpeed=0.0f){
        auto time = std::chrono::system_clock::now();
        initTime = time;
        this->sign = SIGN(distance);
        this->signCorrected.wheelDistance = this->sign * robot.getTranslationalPosition();
        this->signCorrected.initialSpeed = this->sign * initialSpeed;
        this->signCorrected.endSpeed = this->sign * endSpeed;

        this->accTime = (maxSpeed - this->signCorrected.initialSpeed)/this->maxAcc;
        this->decTime = (maxSpeed - this->signCorrected.endSpeed)/this->maxDec;

        this->signCorrected.accDistance = this->signCorrected.initialSpeed * abs(this->accTime) + SIGN(this->accTime) * this->maxAcc * pow(this->accTime, 2)/2.0f;
        this->signCorrected.decDistance = this->maxSpeed * abs(this->decTime) - SIGN(this->decTime) * this->maxDec * pow(this->decTime, 2)/2.0;

        if(this->signCorrected.accDistance + this->signCorrected.decDistance <= abs(distance)){
            this->steTime = (abs(distance) - this->signCorrected.accDistance - this->signCorrected.decDistance)/maxSpeed;
            this->steSpeed = this->sign * maxSpeed;
        }else{
            this->accTime = - this->signCorrected.initialSpeed/this->maxAcc + sqrt((this->maxAcc+this->maxDec) * (2*this->maxAcc*maxDec*abs(distance)+this->maxAcc*pow(this->signCorrected.endSpeed, 2)+this->maxDec*pow(this->signCorrected.initialSpeed,2)));
            this->decTime = (this->maxAcc * this->accTime + this->signCorrected.initialSpeed - this->signCorrected.endSpeed)/this->maxDec;
            this->steTime = 0;
            this->steSpeed = this->signCorrected.initialSpeed + this->maxAcc*this->accTime;

            this->signCorrected.accDistance = this->signCorrected.initialSpeed * abs(this->accTime) + SIGN(this->accTime) * this->maxAcc * pow(this->accTime, 2)/2.0f;
            this->signCorrected.decDistance = this->maxSpeed * abs(this->decTime) - SIGN(this->decTime) * this->maxDec * pow(this->decTime, 2)/2.0;
        }
        this->signCorrected.pointDistance = abs(distance);
        this->initSteTime = this->steTime;
    }



};


#endif //TEENSYCODE2_0_RAMP_H
