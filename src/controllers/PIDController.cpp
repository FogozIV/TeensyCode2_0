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
    std::shared_ptr<Print> logger = robot.getLogger();
    double angle = WARP_ANGLE(delta_pos.getDistanceAngle() - current_pos.getAngle());
    double sign = (angle < -M_PI_2 || angle > M_PI_2) ? -1 : 1;
    distance_target += quadRampDistance->compute(sign*delta_pos.getDistance(), distance_target - robot.getTranslationalPosition());
    double distance_result = distancePID->compute(distance_target - robot.getTranslationalPosition());
    robot.getLoggerMutex()->lock();
    logger->println("Distance ");
    logger->printf("%lf; %lf; %lf\n", distance_target, robot.getTranslationalPosition(), distance_result);
    angle += sign < 0 ? M_PI : 0;
    angle = WARP_ANGLE(angle);
    angle_target += quadRampAngle->compute(angle, angle_target-robot.getZAxisRotation());
    double angle_result = anglePID->compute(angle_target - robot.getZAxisRotation());
    logger->println("Angle ");
    logger->printf("%lf; %lf; ", delta_pos.getDistanceAngle(), current_pos.getAngle());
    logger->printf("%lf; %lf; %lf; %lf\n", angle, angle_target, robot.getZAxisRotation(), angle_result);
    logger->println(current_pos);
    logger->println(delta_pos);
    /*
    logger->print("Current ");
    logger->println(current_pos);
    logger->print("Delta ");
    logger->println(delta_pos);
    logger->printf("%f, %f, %f, %f, %f, %f\n", angle, sign, distance_target, distance_result, angle_target, angle_result);
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
    robot.getLoggerMutex()->unlock();
    robot.applyMotor(std::vector<double>({distance_result+angle_result, distance_result-angle_result}));
    /*ThetaAngleRobotImpl* robotImpl = (ThetaAngleRobotImpl*)(&robot);
    robotImpl->getRightMotor().setPWM(distance_result+angle_result);
    robotImpl->getLeftMotor().setPWM(distance_result-angle_result);*/
}

void PIDController::reset_to(AbstractRobot& robot, const Position &position) {
    distance_target = robot.getTranslationalPosition();
    angle_target = robot.getZAxisRotation();
    anglePID->reset();
    distancePID->reset();
    quadRampAngle->reset();
    quadRampDistance->reset();
}
