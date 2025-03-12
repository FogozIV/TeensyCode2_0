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
    /**
     * Constructor
     * @param pwmPin the pin to send a PWM signal to
     * @param dirPin the pin to send a DIRECTION signal to
     * @param inversed whether a negative pwm send a HIGH to the DIR_PIN or a LOW signal
     */
    DirPWMMotor(uint8_t pwmPin, uint8_t dirPin, bool inversed=false);

    void setPWM(double pwm) override;

    double getMaxValue() const override;

    double getCurrentValue() const override;

    void setReversed(bool) override;

    bool isReversed() const override;

    void setPWM(int pwm) override;
};


#endif //TEENSYCODE2_0_DIRPWMMOTOR_H
