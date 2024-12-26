//
// Created by fogoz on 26/12/2024.
//

#ifndef TEENSYCODE2_0_HOMOGENEOUSMATRIXROBOTIMPL_H
#define TEENSYCODE2_0_HOMOGENEOUSMATRIXROBOTIMPL_H

#include "AbstractRobot.h"
#include "utils/SpeedEstimator.h"
#include "motors/AbstractMotor.h"
#include "encoders/AbstractEncoder.h"

class HomogeneousMatrixRobotImpl: public AbstractRobot {
    std::unique_ptr<AbstractEncoder> leftEncoder;
    std::unique_ptr<AbstractEncoder> rightEncoder;
    std::unique_ptr<AbstractMotor> leftMotor;
    std::unique_ptr<AbstractMotor> rightMotor;
    std::unique_ptr<Matrix<double>> positionMatrix;
    std::unique_ptr<SpeedEstimator> distanceEstimator;
    std::unique_ptr<SpeedEstimator> angleEstimator;
public:
    HomogeneousMatrixRobotImpl(std::unique_ptr<AbstractEncoder> &leftEncoder,
                               std::unique_ptr<AbstractEncoder> &rightEncoder,
                               std::unique_ptr<AbstractMotor> &leftMotor,
                               std::unique_ptr<AbstractMotor> &rightMotor,
                               double bandwidth_distance,
                               double bandwidth_angle);
};


#endif //TEENSYCODE2_0_HOMOGENEOUSMATRIXROBOTIMPL_H
