//
// Created by fogoz on 26/12/2024.
//

#include <FS.h>
#include <SD.h>
#include "robot/ThetaAngleRobotImpl.h"

#include <memory>

ThetaAngleRobotImpl::ThetaAngleRobotImpl(std::unique_ptr<AbstractEncoder> &&leftEncoder,
                                         std::unique_ptr<AbstractEncoder> &&rightEncoder,
                                         std::unique_ptr<AbstractMotor> &&leftMotor,
                                         std::unique_ptr<AbstractMotor> &&rightMotor,
                                         double bandwidth_distance,
                                         double bandwidth_angle)
        : leftEncoder(std::move(leftEncoder)), rightEncoder(std::move(rightEncoder)), leftMotor(std::move(leftMotor)), rightMotor(std::move(rightMotor)) {
    this->distanceEstimator = std::make_unique<SpeedEstimator>(bandwidth_distance);
    this->angleEstimator = std::make_unique<SpeedEstimator>(bandwidth_angle);

    sd_present = SD.begin(SELECTED_CHIP);
    if(sd_present){
        if (File dataFile = SD.open("encoder.json", FILE_READ)) {
            deserializeJson(jsonData, dataFile);
            if (jsonData["left_wheel_diam"].is<double>()) {
                left_wheel_diam = jsonData["left_wheel_diam"].as<double>();
            }
            if (jsonData["right_wheel_diam"].is<double>()) {
                right_wheel_diam = jsonData["right_wheel_diam"].as<double>();
            }
            if (jsonData["track_mm"].is<double>()) {
                track_mm = jsonData["track_mm"].as<double>();
            }
        }
    }



}

bool ThetaAngleRobotImpl::isMoving() const {
    return false;
}

Position ThetaAngleRobotImpl::getPosition() const {
    return pos;
}

const double ThetaAngleRobotImpl::getTranslationalSpeed() const {
    return this->distanceEstimator->getSpeed();
}

const double ThetaAngleRobotImpl::getZAxisRotation() const {
    return this->angleEstimator->getRealDistance();
}

const double ThetaAngleRobotImpl::getRotationalSpeed() const {
    return this->angleEstimator->getSpeed();
}

const double ThetaAngleRobotImpl::getTranslationalPosition() const {
    return this->distanceEstimator->getRealDistance();
}

void ThetaAngleRobotImpl::update_controller() {

}

void ThetaAngleRobotImpl::update() {
    update_position();
    update_controller();
}

void ThetaAngleRobotImpl::begin_calibration() {
    calibration_data.left_position = this->leftEncoder->getEncoderCount();
    calibration_data.right_position = this->rightEncoder->getEncoderCount();
}
void ThetaAngleRobotImpl::save() {
    if (!sd_present) {
        return;
    }
    jsonData["left_wheel_diam"] = left_wheel_diam;
    jsonData["right_wheel_diam"] = right_wheel_diam;
    jsonData["track_mm"] = track_mm;
    File file = SD.open("encoder.json", FILE_WRITE_BEGIN);
    serializeJson(jsonData, file);
    file.close();
}

void ThetaAngleRobotImpl::end_calibration_angle(double angle) {
    double left = this->leftEncoder->getEncoderCount() - calibration_data.left_position;
    double right = this->rightEncoder->getEncoderCount() - calibration_data.right_position;
    double left_c = left*left_wheel_diam;
    double right_c = right*right_wheel_diam;

    double estimated_angle = (right_c - left_c)/track_mm;
    double real_angle = angle * DEG_TO_RAD;

    double multiplier = estimated_angle /real_angle;
    track_mm *= multiplier;
    save();
}

void ThetaAngleRobotImpl::end_calibration_straight(double distance) {
    //(left+right)/2 = distance
    //left == right
    double left = this->leftEncoder->getEncoderCount() - calibration_data.left_position;
    double right = this->rightEncoder->getEncoderCount() - calibration_data.right_position;
    double left_c = left*left_wheel_diam;
    double right_c = right*right_wheel_diam;
    if (left_c *distance < 0) {
        left_c = -left_c;
        left_wheel_diam *= -1;
    }
    if (right_c *distance < 0) {
        right_c = -right_c;
        right_wheel_diam *= -1;
    }
    double multiplier = left_c/right_c;
    right_wheel_diam *= multiplier;
    right_c *= multiplier;

    double estimated_distance = (left_c + right_c)/2;
    double distance_multiplier = distance /estimated_distance;
    left_wheel_diam *= distance_multiplier;
    right_wheel_diam *= distance_multiplier;
    save();
}

void ThetaAngleRobotImpl::update_position() {
    double left = leftEncoder->getDeltaCount() * left_wheel_diam;
    double right = rightEncoder->getDeltaCount() * right_wheel_diam;

    double distance = (left + right)/2;
    double angle = (right-left)/track_mm;
    //Internal count keeper
    this->angleEstimator->update(angle);
    this->distanceEstimator->update(distance);
    if (angle == 0) {
        pos += {cos(pos.getAngle()) * distance, sin(pos.getAngle()) * distance};
    }else {
        double r = distance*track_mm/(right-left);
        Serial.println(r);
        pos+= {r * (-sin(pos.getAngle()) + sin(pos.getAngle() + angle)), r * (cos(pos.getAngle()) - cos(pos.getAngle() + angle)), angle};
    }
}
