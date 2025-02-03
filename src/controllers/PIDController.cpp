//
// Created by fogoz on 03/01/2025.
//

#include "controllers/PIDController.h"
#include "robot/ThetaAngleRobotImpl.h"

PIDController::PIDController(std::shared_ptr<PID> distancePID, std::shared_ptr<PID> anglePID, std::shared_ptr<QuadRamp> rampDistance, std::shared_ptr<QuadRamp> rampAngle):
    distancePID(std::move(distancePID)),
    anglePID(std::move(anglePID)),
    quadRampDistance(std::move(rampDistance)),
    quadRampAngle(std::move(rampAngle)){
}

void PIDController::applyController(AbstractRobot &robot, const Position &target_pos) {
    Position current_pos = robot.getPosition();
    Position delta_pos = target_pos - current_pos;
    double angle = WARP_ANGLE(delta_pos.getDistanceAngle() - robot.getPosition().getAngle());
    double sign = (angle < -90 || angle > 90) ? -1 : 1;
    distance_target = quadRampDistance->compute(sign*delta_pos.getDistance());
    double distance_result = distancePID->compute(distance_target);
    angle_target = quadRampAngle->compute(angle);
    double angle_result = anglePID->compute(angle_target);
    std::shared_ptr<Print> logger = robot.getLogger();
    /*
    logger->print("Current ");
    logger->println(current_pos);
    logger->flush();
    logger->print("Delta ");
    logger->println(delta_pos);
    logger->flush();
    logger->printf("%f, %f, %f, %f, %f, %f\n", angle, sign, distance_target, distance_result, angle_target, angle_result);
    logger->flush();
    */
    logger->print(distance_result+angle_result);
    logger->print(";");
    logger->print(distance_result-angle_result);
    logger->print(";");
    logger->print(robot.getTranslationalPosition());
    logger->print(";");
    logger->println(robot.getZAxisRotation());
    //logger->printf("%f; %f; %f; %f\n", distance_result+angle_result, distance_result-angle_result, robot.getTranslationalPosition(), robot.getZAxisRotation());
    //This goes out of the loop just because we want to be able to work with any type of robot check what can be done for that
    Serial.println("Here");
    ThetaAngleRobotImpl* robotImpl = (ThetaAngleRobotImpl*)(&robot);
    robotImpl->getRightMotor().setPWM(distance_result+angle_result);
    robotImpl->getLeftMotor().setPWM(distance_result-angle_result);
}

void PIDController::reset_to(const AbstractRobot& robot, const Position &position) {
    distance_target = robot.getTranslationalPosition();
    angle_target = robot.getZAxisRotation();
    anglePID->reset();
    distancePID->reset();
    quadRampAngle->reset();
    quadRampDistance->reset();
}
