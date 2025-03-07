//
// Created by fogoz on 02/01/2025.
//

#include "./utils/PID.h"
#include "Arduino.h"

double PID::compute(double error) {
    auto time = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt = time - previousTime;
    previousTime = time;
    iTerm += ki * error * dt.count();
    iTerm = constrain(iTerm, -anti_windup, anti_windup);
    double result = kp * error + iTerm + kd * (error - last_error) / dt.count();
    last_error = error;
    return result;
}


PID::PID(double kp, double ki, double kd, double anti_windup, double max_value) : kp(kp), ki(ki), kd(kd), anti_windup(anti_windup), max_value(max_value) {
}

void PID::reset() {
    last_error = 0;
    iTerm = 0;
    previousTime = std::chrono::steady_clock::now();
}

void PID::set(double kp, double ki, double kd, double anti_windup, double max_value) {
    this->kp = isnan(kp) ? this->kp : kp;
    this->ki = isnan(ki) ? this->ki : ki;
    this->kd = isnan(kd) ? this->kd : kd;
    this->anti_windup = isnan(anti_windup) ? this->anti_windup : anti_windup;
    this->max_value = isnan(max_value) ? this->max_value : max_value;

}

double PID::getKp() const {
    return kp;
}

double PID::getKi() const {
    return ki;
}

double PID::getKd() const {
    return kd;
}

double PID::getAntiWindup() const {
    return anti_windup;
}

double PID::getMaxValue() const {
    return max_value;
}
