//
// Created by fogoz on 26/12/2024.
//

#ifndef TEENSYCODE2_0_HOMOGENEOUSMATRIXROBOTIMPL_H
#define TEENSYCODE2_0_HOMOGENEOUSMATRIXROBOTIMPL_H

#include <memory>

#include "utils/Position.h"
#include "AbstractRobot.h"
#include "utils/SpeedEstimator.h"
#include "motors/AbstractMotor.h"
#include "encoders/AbstractEncoder.h"
#include "ArduinoJson.h"

#ifndef SELECTED_CHIP
#define SELECTED_CHIP BUILTIN_SDCARD
#endif
class ThetaAngleRobotImpl: public AbstractRobot {
    std::shared_ptr<AbstractEncoder> leftEncoder;
    std::shared_ptr<AbstractEncoder> rightEncoder;
    std::shared_ptr<AbstractMotor> leftMotor;
    std::shared_ptr<AbstractMotor> rightMotor;
    std::shared_ptr<SpeedEstimator> distanceEstimator;
    std::shared_ptr<SpeedEstimator> angleEstimator;
    Position pos = {0,0,0};
    double left_wheel_diam = 1;
    double right_wheel_diam = 1;
    double track_mm = 100;

    bool left_wheel_motor_reversed = false;
    bool right_wheel_motor_reversed = false;
    struct {
        double left_position = 0.0f;
        double right_position = 0.0f;
    }calibration_data;

    bool sd_present;
    JsonDocument jsonData;
protected:
    void update_position();
    void update_controller();
public:
    ThetaAngleRobotImpl(std::shared_ptr<AbstractEncoder> leftEncoder,
                               std::shared_ptr<AbstractEncoder> rightEncoder,
                               std::shared_ptr<AbstractMotor> leftMotor,
                               std::shared_ptr<AbstractMotor> rightMotor,
                               double bandwidth_distance,
                               double bandwidth_angle);

    bool isMoving() const override;

    const double getTranslationalSpeed() const override;

    const double getZAxisRotation() const override;

    const double getRotationalSpeed() const override;

    const double getTranslationalPosition() const override;

    void update() override;

    void begin_calibration() override;

    void end_calibration_angle(double angle) override;

    void end_calibration_straight(double distance) override;

    void save();

    Position getPosition() const override;

    void find_motor_calibration() override;
};



#endif //TEENSYCODE2_0_HOMOGENEOUSMATRIXROBOTIMPL_H
