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
    bool inversed;
    double currentValue = 0;
public:
    DirPWMMotor(uint8_t pwmPin, uint8_t dirPin, bool inversed=false);

    void setPWM(double pwm) override;

    double getMaxValue() const override;

    double getCurrentValue() const override;

    void setReversed(bool) override;

    bool isReversed() const override;
};


#endif //TEENSYCODE2_0_DIRPWMMOTOR_H
