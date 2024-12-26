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
    bool isMoving() const;

    /**
     * @return the position matrix
     */
    Matrix<double> getPosition() const;

    /**
     * @return the speed in the translation
     */
    double getTranslationalSpeed() const;

    /**
     * @return the rotation along the z axis (which is following the robot)
     */
    const double getZAxisRotation() const;

    /**
     * @return the speed along the rotation axis
     */
    double getRotationalSpeed() const;

};


#endif //TEENSYCODE2_0_ABSTRACTROBOT_H
