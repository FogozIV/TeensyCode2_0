//
// Created by fogoz on 26/12/2024.
//

#include <FS.h>
#include <SD.h>
#include "robot/HomogeneousMatrixRobotImpl.h"

HomogeneousMatrixRobotImpl::HomogeneousMatrixRobotImpl(std::unique_ptr<AbstractEncoder> &&leftEncoder,
                                                       std::unique_ptr<AbstractEncoder> &&rightEncoder,
                                                       std::unique_ptr<AbstractMotor> &&leftMotor,
                                                       std::unique_ptr<AbstractMotor> &&rightMotor,
                                                       double bandwidth_distance,
                                                       double bandwidth_angle)
        : leftEncoder(std::move(leftEncoder)), rightEncoder(std::move(rightEncoder)), leftMotor(std::move(leftMotor)), rightMotor(std::move(rightMotor)) {
    this->positionMatrix = std::make_unique<Matrix<double>>(Matrix<double>::getEye(4));
    this->distanceEstimator = std::make_unique<SpeedEstimator>(bandwidth_distance);
    this->angleEstimator = std::make_unique<SpeedEstimator>(bandwidth_angle);

    Serial.println("HomogeneousMatrixRobotImpl constructor");
    sd_present = SD.begin(SELECTED_CHIP);
    if(sd_present){
        File dataFile = SD.open("encoder.json", FILE_READ);
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

bool HomogeneousMatrixRobotImpl::isMoving() const {
    return false;
}

Matrix<double> HomogeneousMatrixRobotImpl::getPosition() const {
    return *positionMatrix.get();
}

const double HomogeneousMatrixRobotImpl::getTranslationalSpeed() const {
    return this->distanceEstimator->getSpeed();
}

const double HomogeneousMatrixRobotImpl::getZAxisRotation() const {
    return this->angleEstimator->getRealDistance();
}

const double HomogeneousMatrixRobotImpl::getRotationalSpeed() const {
    return this->angleEstimator->getSpeed();
}

const double HomogeneousMatrixRobotImpl::getTranslationalPosition() const {
    return this->distanceEstimator->getRealDistance();
}

void HomogeneousMatrixRobotImpl::update_controller() {

}

void HomogeneousMatrixRobotImpl::update() {
    update_position();
    update_controller();
}

void HomogeneousMatrixRobotImpl::update_position() {
    double left = leftEncoder->getDeltaCount() * left_wheel_diam;
    double right = rightEncoder->getDeltaCount() * right_wheel_diam;

    double distance = (left + right)/2;
    double angle = (right-left)/track_mm;
    //Internal count keeper
    this->angleEstimator->update(angle);
    this->distanceEstimator->update(distance);
    if (angle == 0) {
        *(this->positionMatrix) *= Matrix<double>::getTranslation(distance);
    }else {
        double r = distance*track_mm/(right-left);
        //Rotation autour d'un point en (0,-r) dans le repère local
        *(this->positionMatrix) *= Matrix<double>::getTranslation(0, r);
        *(this->positionMatrix) *= Matrix<double>::getZRot(angle);
        *(this->positionMatrix) *= Matrix<double>::getTranslation(0, -r);
    }

}
