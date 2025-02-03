//
// Created by fogoz on 22/12/2024.
//

#ifndef TEENSYCODE2_0_ABSTRACTROBOT_H
#define TEENSYCODE2_0_ABSTRACTROBOT_H
#include "utils/Position.h"

class AbstractRobot {
public:
    virtual ~AbstractRobot() = default;

    /**
     * @return whether the robot is moving or not
     */
    virtual bool isMoving() const = 0;

    /**
     * @return the position matrix
     */
    virtual Position getPosition() const = 0;

    /**
     * @return the total translational distance
     */
    virtual const double getTranslationalPosition() const = 0;
    /**
     * @return the speed in the translation
     */
    virtual const double getTranslationalSpeed() const = 0;

    /**
     * @return the rotation along the z axis (which is following the robot)
     */
    virtual const double getZAxisRotation() const = 0;

    /**
     * @return the speed along the rotation axis
     */
    virtual const double getRotationalSpeed() const = 0;

    /**
     * Function called all the time to update the robot pos and control
     */
    virtual void update()= 0;

    /**
     * Function to call at the beginning of a calibration
     */
    virtual void begin_calibration() = 0;

    /**
     * Function to call to calibrate the motor
     */
    virtual void find_motor_calibration() = 0;

    /**
     * Function to call at the end of a calibration
     * @param angle the angle that has been moved
     */
    virtual void end_calibration_angle(double angle) = 0;

    /**
     * Function to call at the end of a calibration
     * @param distance the distance moved
     */
    virtual void end_calibration_straight(double distance) = 0;

    /**
     *
     * @param position the new_position for everything
     */
    virtual void reset_robot_to(const Position& position) = 0;


    /**
     *
     * @param position the target position
     */
    virtual void setTargetPos(const Position& position) = 0;

    virtual std::shared_ptr<Print> getLogger() = 0;

};


#endif //TEENSYCODE2_0_ABSTRACTROBOT_H
