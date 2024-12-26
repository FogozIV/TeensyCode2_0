//
// Created by fogoz on 26/12/2024.
//

#ifndef TEENSYCODE2_0_SPEEDESTIMATOR_H
#define TEENSYCODE2_0_SPEEDESTIMATOR_H

//Based on PLLEstimator
//https://discourse.odriverobotics.com/t/rotor-encoder-pll-and-velocity/224/2
//https://github.com/EsialRobotik/asserv_chibios/blob/master/src/Pll.cpp
class SpeedEstimator {
    double kp;
    double ki;
    double speed;
    double distance_estimation;
    double real_distance;
public:
    SpeedEstimator(double bandwidth);

    void update(double distance, double deltaT);

    void reset();

    void setBandwidth(double bandwidth);

};


#endif //TEENSYCODE2_0_SPEEDESTIMATOR_H
