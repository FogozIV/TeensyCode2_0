//
// Created by fogoz on 26/12/2024.
//

#include "robot/ThetaAngleRobotImpl.h"

#include <memory>
#include <utility>
#include <Entropy.h>
ThetaAngleRobotImpl::ThetaAngleRobotImpl(std::shared_ptr<AbstractEncoder> leftEncoder,
                                         std::shared_ptr<AbstractEncoder> rightEncoder,
                                         std::shared_ptr<AbstractMotor> leftMotor,
                                         std::shared_ptr<AbstractMotor> rightMotor,
                                         std::shared_ptr<AbstractController> controller,
                                         double bandwidth_distance,
                                         double bandwidth_angle)
        : leftEncoder(std::move(leftEncoder)), rightEncoder(std::move(rightEncoder)), leftMotor(std::move(leftMotor)), rightMotor(std::move(rightMotor)), controller(std::move(controller)){

    this->distanceEstimator = std::make_shared<SpeedEstimator>(bandwidth_distance);
    this->angleEstimator = std::make_shared<SpeedEstimator>(bandwidth_angle);
    sd_present = SD.begin(SELECTED_CHIP);
    if(sd_present){
        Entropy.Initialize();

        this->data_file = SD.open((std::string("data") + std::to_string(Entropy.random(0,255))).data(), FILE_WRITE_BEGIN);
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
            if (jsonData["left_wheel_motor_reversed"].is<bool>()) {
                left_wheel_motor_reversed = jsonData["left_wheel_motor_reversed"].as<bool>();
            }
            if (jsonData["right_wheel_motor_reversed"].is<bool>()) {
                right_wheel_motor_reversed = jsonData["right_wheel_motor_reversed"].as<bool>();
            }
            this->leftMotor->setReversed(left_wheel_motor_reversed);
            this->rightMotor->setReversed(right_wheel_motor_reversed);
            if (jsonData["reverse_motors"].is<bool>() && jsonData["reverse_motors"].as<bool>()) {
                auto a = this->leftMotor;
                this->leftMotor = this->rightMotor;
                this->rightMotor = a;
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

void ThetaAngleRobotImpl::find_motor_calibration() {
    leftMotor->setPWM(0);
    rightMotor->setPWM(0);
    delay(1000);
    angleEstimator->reset();
    distanceEstimator->reset();
    //We go back in a previsible state
    if (jsonData["reverse_motors"].is<bool>() && jsonData["reverse_motors"].as<bool>()) {
        auto a = leftMotor;
        leftMotor = rightMotor;
        rightMotor = a;
    }
    pos = {0,0,0};

    leftMotor->setReversed(false);
    rightMotor->setReversed(false);

    leftMotor->setPWM(leftMotor->getMaxValue() * 0.1);
    delay(2000);
    leftMotor->setPWM(0);

    update_position();


    if (distanceEstimator->getRealDistance() > 0) {
        jsonData["left_wheel_motor_reversed"] = false;
        if (angleEstimator->getRealDistance() < 0) {
            jsonData["reverse_motors"] = false;
        }else {
            jsonData["reverse_motors"] = true;
        }
    }else {
        jsonData["left_wheel_motor_reversed"] = true;
        //Case where the wheel went backward
        if (angleEstimator->getRealDistance() > 0) {
            jsonData["reverse_motors"] = true;
        }else {
            jsonData["reverse_motors"] = false;
        }
    }
    angleEstimator->reset();
    distanceEstimator->reset();

    rightMotor->setPWM(rightMotor->getMaxValue() * 0.1);
    delay(2000);
    rightMotor->setPWM(0);
    update_position();

    if (distanceEstimator->getRealDistance() > 0) {
        jsonData["right_wheel_motor_reversed"] = false;
    }else {
        jsonData["right_wheel_motor_reversed"] = true;
    }

    leftMotor->setReversed(jsonData["left_wheel_motor_reversed"].as<bool>());
    rightMotor->setReversed(jsonData["right_wheel_motor_reversed"].as<bool>());

    if (jsonData["reverse_motors"].is<bool>() && jsonData["reverse_motors"].as<bool>()) {
        auto a = leftMotor;
        leftMotor = rightMotor;
        rightMotor = a;
    }
    pos = {0,0,0};
    angleEstimator->reset();
    distanceEstimator->reset();


    leftMotor->setPWM(-leftMotor->getMaxValue() * 0.1);
    rightMotor->setPWM(rightMotor->getMaxValue() * 0.1);
    delay(2000);
    leftMotor->setPWM(0);
    rightMotor->setPWM(0);

    save();
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
    controller->applyController(*this, target_pos);
    if(sd_present){
        this->data_file.printf("%f, %f\n", getTranslationalPosition(), getZAxisRotation());
        this->data_file.flush();
    }
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

void ThetaAngleRobotImpl::reset_robot_to(const Position &position) {
    this->pos = position;
    this->target_pos = position;
    this->controller->reset_to(*this, this->pos);
}

void ThetaAngleRobotImpl::setTargetPos(const Position &position) {
    this->target_pos = position;
}

Print& ThetaAngleRobotImpl::getLogger(){
    if(sd_present){
        return this->data_file;
    }
    return Serial;
}

AbstractMotor &ThetaAngleRobotImpl::getRightMotor() {
    return *rightMotor;
}

AbstractMotor &ThetaAngleRobotImpl::getLeftMotor() {
    return *leftMotor;
}
