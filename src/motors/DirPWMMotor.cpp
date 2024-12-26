//
// Created by fogoz on 26/12/2024.
//

#include "motors/DirPWMMotor.h"

#ifndef NATIVE
#include "Arduino.h"
#endif

void DirPWMMotor::setPWM(double pwm) {
    digitalWriteFast(dirPin, (inversed ? -1 : 1) * pwm > 0);
    analogWrite(pwmPin, (uint16_t)abs(pwm));
}

double DirPWMMotor::getMaxValue() const {
    return pow(2,12)-1;
}

double DirPWMMotor::getCurrentValue() const {
    return currentValue;
}

DirPWMMotor::DirPWMMotor(uint8_t pwmPin, uint8_t dirPin, bool inversed) : pwmPin(pwmPin), dirPin(dirPin), inversed(inversed) {
#ifndef NATIVE
    analogWriteResolution(12);
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    analogWrite(pwmPin, 0);
    digitalWriteFast(dirPin, LOW);
#endif
}
