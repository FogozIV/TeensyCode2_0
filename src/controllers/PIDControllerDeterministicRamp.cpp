//
// Created by fogoz on 17/02/2025.
//

#include "controllers/PIDControllerDeterministicRamp.h"

PIDControllerDeterministicRamp::PIDControllerDeterministicRamp(std::shared_ptr<PID> distancePID,
                                                               std::shared_ptr<PID> anglePID,
                                                               std::shared_ptr<Ramp> rampDistance) : distancePID(distancePID),
                                                                                                     anglePID(anglePID), rampDistance(rampDistance){

}

void PIDControllerDeterministicRamp::applyController(AbstractRobot &robot, const Position &target_pos) {
    if(target_pos != pos){
        setTargetPos(robot, target_pos);
    }
    Position current_pos = robot.getPosition();
    Position delta_pos = target_pos - current_pos;
    std::shared_ptr<Print> logger = robot.getLogger();
    double angle = WARP_ANGLE(delta_pos.getDistanceAngle() - current_pos.getAngle());
    double sign = (angle < -M_PI_2 || angle > M_PI_2) ? -1 : 1;
    RampReturnData ramp = rampDistance->compute(robot, (target_pos-current_pos).getDistance());
    distance_target += ramp.distance_increment;
    double distance_result = distancePID->compute(distance_target - robot.getTranslationalPosition());
    logger->println("Distance ");
    logger->flush();
    logger->printf("%lf; %lf; %lf\n", distance_target, robot.getTranslationalPosition(), distance_result);
    angle += sign < 0 ? M_PI : 0;
    angle = WARP_ANGLE(angle);
    double angle_result = anglePID->compute(angle_target);
    logger->println("Angle ");
    logger->printf("%lf; %lf; ", delta_pos.getDistanceAngle(), current_pos.getAngle());
    logger->printf("%lf; %lf; %lf; %lf\n", angle, angle_target, robot.getZAxisRotation(), angle_result);
    logger->println(current_pos);
    logger->println(delta_pos);
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
    robot.applyMotor({distance_result-angle_result, distance_result+angle_result});

}

void PIDControllerDeterministicRamp::reset_to(AbstractRobot &robot, const Position &position) {
    rampDistance->reset();
    distance_target = robot.getTranslationalPosition();
    angle_target = robot.getZAxisRotation();
    anglePID->reset();
    distancePID->reset();
}

void PIDControllerDeterministicRamp::setTargetPos(AbstractRobot &robot, const Position &position) {
    rampDistance->setTarget(robot, (position-robot.getPosition()).getDistance(), robot.getTranslationalSpeed());
}
