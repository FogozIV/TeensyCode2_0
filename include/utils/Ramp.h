//
// Created by fogoz on 11/02/2025.
//

#ifndef TEENSYCODE2_0_RAMP_H
#define TEENSYCODE2_0_RAMP_H

#include "../robot/AbstractRobot.h"
#include "chrono"

#define SIGN(x) (x < 0 ? -1 : 1)

struct RampReturnData{
    bool end;
    double distance_increment;
    double speed;
    bool stop;
    bool started;
};
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


    std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<double>> initTime;

    Ramp::Private previousComputation = {false, NAN, NAN};

    double accTime;
    double decTime;
    double steTime;

    double initSteTime;

    double steSpeed;

    bool started = false;


public:
    Ramp(double acc, double dec, double maxSpeed) : maxAcc(acc), maxDec(dec), maxSpeed(maxSpeed){
    }

    void setTarget(const AbstractRobot& robot, double distance, double initialSpeed=0.0f, double endSpeed=0.0f){
        auto time = std::chrono::steady_clock::now();
        if(!started ||(time- initTime) > std::chrono::milliseconds(20)){
            robot.getLogger()->println("DATA");
            robot.getLogger()->println(time.time_since_epoch().count());
            robot.getLogger()->println(initTime.time_since_epoch().count());
            initTime = time;
        }
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
            robot.getLogger()->println("TRIANGLE RAMP");
            this->steSpeed = sqrt((2*maxAcc * abs(distance) * maxDec + maxAcc * pow(signCorrected.endSpeed, 2) + maxDec * pow(signCorrected.initialSpeed, 2))/(maxAcc+maxDec));
            this->accTime = (this->steSpeed - signCorrected.initialSpeed)/maxAcc;
            //this->accTime = - this->signCorrected.initialSpeed/this->maxAcc + sqrt((this->maxAcc+this->maxDec) * (2*this->maxAcc*maxDec*abs(distance)+this->maxAcc*pow(this->signCorrected.endSpeed, 2)+this->maxDec*pow(this->signCorrected.initialSpeed,2)));
            this->decTime = (this->steSpeed - signCorrected.endSpeed)/maxDec;
            this->signCorrected.accDistance = this->signCorrected.initialSpeed * abs(this->accTime) + SIGN(this->accTime) * this->maxAcc * pow(this->accTime, 2)/2.0f;
            this->signCorrected.decDistance = this->steSpeed * abs(this->decTime) - SIGN(this->decTime) * this->maxDec * pow(this->decTime, 2)/2.0;
            robot.getLogger()->println(this->signCorrected.accDistance);
            robot.getLogger()->println(this->signCorrected.decDistance);
        }
        this->signCorrected.pointDistance = abs(distance);
        this->initSteTime = this->steTime;
        this->signCorrected.wheelDistance = sign*robot.getTranslationalPosition();
        started = true;
        robot.getLogger()->println("RAMP");
        robot.getLogger()->println(accTime);
        robot.getLogger()->println(steTime);
        robot.getLogger()->println(decTime);
        robot.getLogger()->flush();
    }

    RampReturnData compute(const AbstractRobot& robot, double distance){
        if(increment)
            updateSTETime(robot, distance);
        if(!started)
            return {false, 0, false, false};
        double time = std::chrono::duration<double>(std::chrono::steady_clock::now() - initTime).count();
        robot.getLogger()->println(time);
        Private data = computeAtTime(time);
        RampReturnData returnData;
        if(std::isnan(previousComputation.speed) || std::isnan(previousComputation.distance)){
            returnData = {data.done, data.distance, data.speed, data.done && data.speed == 0.0f, true};
        }else{
            returnData = {data.done, data.distance - previousComputation.distance, data.speed, data.done && data.speed == 0.0f, true};
        }
        previousComputation = data;
        return returnData;
    }

    Private computeAtTime(double time){
        double distance;
        double current_speed;
        bool done = false;
        if(time < accTime){
            current_speed = signCorrected.initialSpeed + maxAcc * time;
            distance = signCorrected.initialSpeed*time + maxAcc * pow(time, 2)/2.0f;
        }else if(time < accTime + steTime){
            current_speed = steSpeed;
            distance = signCorrected.initialSpeed * accTime + maxAcc * pow(accTime, 2)/2.0f + (time - accTime) * steSpeed;
        }else if(time < accTime + steTime + decTime){
            current_speed = steSpeed - maxDec*(time - accTime - steTime);
            distance = signCorrected.initialSpeed * accTime + maxAcc * pow(accTime, 2)/2.0f + steTime * steSpeed + steSpeed * (time - accTime - steTime) - maxDec*pow(time - accTime - steTime, 2)/2.0f;
        }else{
            current_speed = this->signCorrected.endSpeed;
            distance = signCorrected.initialSpeed * accTime + maxAcc * pow(accTime, 2)/2.0f + steTime * steSpeed + steSpeed * decTime - maxDec*pow(decTime, 2)/2.0f;
            done = true;
        }
        return {done, current_speed * this->sign, distance * this->sign};
    }

    void updateSTETime(const AbstractRobot& robot, double distance){
        double wheel_distance = robot.getTranslationalPosition();
        auto currentTime = std::chrono::steady_clock::now();
        auto time = (currentTime - initTime).count();
        if(time > accTime && time < accTime + steTime)
            steTime = initSteTime + ((sign * wheel_distance - signCorrected.wheelDistance) + sign * distance - signCorrected.pointDistance)/steSpeed;

    }

    void reset(){
        started = false;
    }




};


#endif //TEENSYCODE2_0_RAMP_H
