//
// Created by fogoz on 26/12/2024.
//

#ifndef TEENSYCODE2_0_HOMOGENEOUSMATRIXROBOTIMPL_H
#define TEENSYCODE2_0_HOMOGENEOUSMATRIXROBOTIMPL_H

#include "AbstractRobot.h"
#include "utils/SpeedEstimator.h"
#include "motors/AbstractMotor.h"
#include "encoders/AbstractEncoder.h"
#include "ArduinoJson.h"

#ifndef SELECTED_CHIP
#define SELECTED_CHIP BUILTIN_SDCARD
#endif
class HomogeneousMatrixRobotImpl: public AbstractRobot {
    std::unique_ptr<AbstractEncoder> leftEncoder;
    std::unique_ptr<AbstractEncoder> rightEncoder;
    std::unique_ptr<AbstractMotor> leftMotor;
    std::unique_ptr<AbstractMotor> rightMotor;
    std::unique_ptr<Matrix<double>> positionMatrix;
    std::unique_ptr<SpeedEstimator> distanceEstimator;
    std::unique_ptr<SpeedEstimator> angleEstimator;

    double left_wheel_diam = 1;
    double right_wheel_diam = 1;
    double track_mm = 100;

    bool sd_present;
    JsonDocument jsonData;
protected:
    void update_position();
    void update_controller();
public:
    HomogeneousMatrixRobotImpl(std::unique_ptr<AbstractEncoder> &&leftEncoder,
                               std::unique_ptr<AbstractEncoder> &&rightEncoder,
                               std::unique_ptr<AbstractMotor> &&leftMotor,
                               std::unique_ptr<AbstractMotor> &&rightMotor,
                               double bandwidth_distance,
                               double bandwidth_angle);

    bool isMoving() const override;

    Matrix<double> getPosition() const override;

    const double getTranslationalSpeed() const override;

    const double getZAxisRotation() const override;

    const double getRotationalSpeed() const override;

    const double getTranslationalPosition() const override;

    void update() override;


};


#endif //TEENSYCODE2_0_HOMOGENEOUSMATRIXROBOTIMPL_H
