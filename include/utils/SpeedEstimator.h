//
// Created by fogoz on 26/12/2024.
//

#ifndef TEENSYCODE2_0_SPEEDESTIMATOR_H
#define TEENSYCODE2_0_SPEEDESTIMATOR_H

#include "chrono"
//Based on PLLEstimator
//https://discourse.odriverobotics.com/t/rotor-encoder-pll-and-velocity/224/2
//https://github.com/EsialRobotik/asserv_chibios/blob/master/src/Pll.cpp
class SpeedEstimator {
    double kp;
    double ki;
    double speed = 0.0f;
    double distance_estimation = 0.0f;
    double real_distance = 0.0f;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> previous_time;

public:
    SpeedEstimator(double bandwidth);

    void update(double distance, double deltaT);

    void update(double distance);

    void reset();

    void setBandwidth(double bandwidth);

    double getSpeed() const;

    double getDistanceEstimation() const;

    double getRealDistance() const;

};


#endif //TEENSYCODE2_0_SPEEDESTIMATOR_H
