//
// Created by fogoz on 26/12/2024.
//

#include "motors/DirPWMMotor.h"

void DirPWMMotor::setPWM(double pwm) {

}

double DirPWMMotor::getMaxValue() const {
    return 0;
}

double DirPWMMotor::getCurrentValue() const {
    return 0;
}

DirPWMMotor::DirPWMMotor(uint8_t pwmPin, uint8_t dirPin) : pwmPin(pwmPin), dirPin(dirPin) {

}
