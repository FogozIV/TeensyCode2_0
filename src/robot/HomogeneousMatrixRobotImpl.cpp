//
// Created by fogoz on 26/12/2024.
//

#include "robot/HomogeneousMatrixRobotImpl.h"

HomogeneousMatrixRobotImpl::HomogeneousMatrixRobotImpl(std::unique_ptr<AbstractEncoder> &leftEncoder,
                                                       std::unique_ptr<AbstractEncoder> &rightEncoder,
                                                       std::unique_ptr<AbstractMotor> &leftMotor,
                                                       std::unique_ptr<AbstractMotor> &rightMotor,
                                                       double bandwidth_distance,
                                                       double bandwidth_angle)
        : leftEncoder(std::move(leftEncoder)), rightEncoder(std::move(rightEncoder)), leftMotor(std::move(leftMotor)), rightMotor(std::move(rightMotor)) {
    this->positionMatrix = std::make_unique<Matrix<double>>(Matrix<double>::getEye(4));
    this->distanceEstimator = std::make_unique<SpeedEstimator>(bandwidth_distance);
    this->angleEstimator = std::make_unique<SpeedEstimator>(bandwidth_angle);
}
