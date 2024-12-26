//
// Created by fogoz on 26/12/2024.
//

#ifndef TEENSYCODE2_0_DIRPWMMOTOR_H
#define TEENSYCODE2_0_DIRPWMMOTOR_H

#include "AbstractMotor.h"
#include "cstdint"

class DirPWMMotor: public AbstractMotor {
    uint8_t pwmPin;
    uint8_t dirPin;
public:
    DirPWMMotor(uint8_t pwmPin, uint8_t dirPin);

    void setPWM(double pwm) override;

    double getMaxValue() const override;

    double getCurrentValue() const override;
};


#endif //TEENSYCODE2_0_DIRPWMMOTOR_H
