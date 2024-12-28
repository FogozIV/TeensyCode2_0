//
// Created by fogoz on 22/12/2024.
//

#ifndef TEENSYCODE2_0_ABSTRACTROBOT_H
#define TEENSYCODE2_0_ABSTRACTROBOT_H
#include "./utils/Matrix.h"

class AbstractRobot {
public:
    /**
     * @return whether the robot is moving or not
     */
    virtual bool isMoving() const = 0;

    /**
     * @return the position matrix
     */
    virtual Matrix<double> getPosition() const = 0;

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

};


#endif //TEENSYCODE2_0_ABSTRACTROBOT_H
