//
// Created by fogoz on 02/01/2025.
//

#ifndef PID_H
#define PID_H

#include <chrono>

class PID {
    double kp, ki, kd;
    double last_error = 0.0f;
    double iTerm = 0.0f;
    double anti_windup;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> previousTime = std::chrono::system_clock::now();
  public:
    /**
     * A PID class
     * @param kp the proportional term
     * @param ki the integral term
     * @param kd the derivative term
     * @param anti_windup the saturation value
     */
    PID(double kp, double ki, double kd, double anti_windup=2000);

    /**
     *
     * @param error the current error
     * @return the
     */
    double compute(double error);

    /**
     * Reset the values
     */
    void reset();
};


#endif //PID_H
