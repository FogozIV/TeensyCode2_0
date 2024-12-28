#include <Arduino.h>

#include "utils/Matrix.h"
#include "vector"
#include "motors/DirPWMMotor.h"
#include "encoders/QuadEncoderImpl.h"
#include "utils/SpeedEstimator.h"
#include "robot/HomogeneousMatrixRobotImpl.h"
std::shared_ptr<AbstractRobot> robot;
void setup() {
    Serial.begin(115200);
    Serial.println("Hello World");
    robot = std::make_shared<HomogeneousMatrixRobotImpl>(
        std::make_unique<QuadEncoderImpl>(0,1,1),
        std::make_unique<QuadEncoderImpl>(2,3,2),
        std::make_unique<DirPWMMotor>(-1,-1),
        std::make_unique<DirPWMMotor>(-1,-1),
        50,20);
    Serial.println("HomogeneousMatrixRobot");
// write your initialization code here
}

void loop() {
    robot->update();
    Serial.println(robot->getPosition());
    Serial.printf("Total distance: %f, total angle: %f, speed distance : %f, speed angle : %f\n",
                  robot->getTranslationalPosition(), robot->getZAxisRotation(), robot->getTranslationalSpeed(),
                  robot->getRotationalSpeed());

}
