//
// Created by fogoz on 26/12/2024.
//

#include "motors/DirPWMMotor.h"

#ifndef NATIVE
#include "Arduino.h"
#endif

void DirPWMMotor::setPWM(double pwm) {
    digitalWriteFast(dirPin, (inversed ? -1 : 1) * pwm > 0);
    analogWrite(pwmPin, static_cast<uint16_t>(abs(constrain(pwm, -getMaxValue(), getMaxValue()))));
}

double DirPWMMotor::getMaxValue() const {
    return 4095.0;
}

double DirPWMMotor::getCurrentValue() const {
    return currentValue;
}

void DirPWMMotor::setReversed(bool reversed) {
    this->inversed = reversed;
}

bool DirPWMMotor::isReversed() const {
    return this->inversed;
}

DirPWMMotor::DirPWMMotor(uint8_t pwmPin, uint8_t dirPin, bool inversed) : pwmPin(pwmPin), dirPin(dirPin), inversed(inversed) {
    if (pwmPin == 255 || dirPin == 255) {
        return;
    }
#ifndef NATIVE
    Serial.println("DirPWMMotor::DirPWMMotor");
    analogWriteResolution(12);
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    analogWrite(pwmPin, 0);
    digitalWriteFast(dirPin, LOW);
#endif
}
