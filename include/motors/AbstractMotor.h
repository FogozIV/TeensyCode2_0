//
// Created by fogoz on 22/12/2024.
//

#ifndef TEENSYCODE2_0_ABSTRACTMOTOR_H
#define TEENSYCODE2_0_ABSTRACTMOTOR_H


class AbstractMotor {
public:
    /**
     * This function should allow the user to send a signed pwm if the value is bigger than the max it will be capped
     * @param pwm a signed value for the pwm signal to put
     */
    virtual void setPWM(double pwm) = 0;
    /**
     * @return the max value we can send otherwise it will be capped
     */
    virtual double getMaxValue() const = 0;
    /**
     * @return the current signed pwm sent to the motor
     */
    virtual double getCurrentValue() const = 0;
};


#endif //TEENSYCODE2_0_ABSTRACTMOTOR_H
