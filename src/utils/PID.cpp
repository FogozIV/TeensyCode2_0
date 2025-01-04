//
// Created by fogoz on 02/01/2025.
//

#include "./utils/PID.h"
#include "Arduino.h"

double PID::compute(double error) {
    double c_error = error;
    auto time = std::chrono::system_clock::now();
    std::chrono::duration<double> dt = time - previousTime;
    previousTime = time;
    iTerm += kp * error * dt.count();
    iTerm = constrain(iTerm, -anti_windup, anti_windup);
    return kp * error + iTerm + kd * (error - last_error) / dt.count();
}


PID::PID(double kp, double ki, double kd, double anti_windup) : kp(kp), ki(ki), kd(kd), anti_windup(anti_windup) {
}

void PID::reset() {
    last_error = 0;
    iTerm = 0;
    previousTime = std::chrono::system_clock::now();
}
