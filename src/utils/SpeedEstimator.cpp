//
// Created by fogoz on 26/12/2024.
//

#include "utils/SpeedEstimator.h"
#include "cmath"
SpeedEstimator::SpeedEstimator(double bandwidth) {
    setBandwidth(bandwidth);
    reset();
}

void SpeedEstimator::setBandwidth(double bandwidth) {
    this->kp = 2.0f * bandwidth;
    this->ki = 0.25f * this->kp * this->kp;
}

void SpeedEstimator::reset() {
    distance_estimation = 0.0f;
    speed = 0.0f;
    real_distance = 0.0f;
}

void SpeedEstimator::update(double distance, double deltaT) {
    distance_estimation += deltaT * speed;

    real_distance += distance;
    double deltaPos = real_distance - distance_estimation;

    distance_estimation += deltaT * kp * deltaPos;
    speed += deltaT * ki * deltaPos;

    if (std::abs(speed) < 0.5f * deltaT * ki)
        speed = 0.0f;
}
