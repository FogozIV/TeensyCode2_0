#include "chrono"
#include "Arduino.h"
//
// Created by fogoz on 02/01/2025.
//

#ifndef QUADRAMP_H
#define QUADRAMP_H

#define SIGN(x) (x < 0 ? -1 : 1)
class QuadRamp{
    double current_speed = 0.0f;

    double maxSpeed;

    double maxAcc;
    double secRatio;

    std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> previous_time;
public:
    QuadRamp(double maxSpeed, double maxAcc, double sec_ratio=1.1f){
        this->maxSpeed = maxSpeed;
        this->maxAcc = maxAcc;
        this->secRatio = sec_ratio;
        this->previous_time = std::chrono::system_clock::now();
    }
    double compute(double target_distance){
        double absStopDistance = pow(current_speed, 2) / (2 * maxAcc);
        double stopDistance = absStopDistance * SIGN(current_speed);
        double target_speed = 0.0f;
        if(abs(target_distance) - absStopDistance*secRatio >= 0){
            target_speed = SIGN(target_distance) * maxSpeed;
        }
        auto current_time = std::chrono::system_clock::now();
        std::chrono::duration<double> dt = current_time - previous_time;
        if(target_speed > current_speed){
            current_speed += maxAcc * dt.count();
        }else if(target_speed < current_speed){
            current_speed -= maxAcc * dt.count();
        }
        current_speed = constrain(current_speed, -maxSpeed, maxSpeed);
        previous_time = current_time;
        return current_speed * dt.count();
    }

    void reset(){
        this->previous_time = std::chrono::system_clock::now();
        this->current_speed = 0.0f;
    }
};

#endif //QUADRAMP_H
