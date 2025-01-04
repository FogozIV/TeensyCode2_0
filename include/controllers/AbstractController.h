//
// Created by fogoz on 30/12/2024.
//

#ifndef ABSTRACTCONTROLLER_H
#define ABSTRACTCONTROLLER_H
#include <tuple>
#include "../robot/AbstractRobot.h"

class AbstractController {
public:
    virtual ~AbstractController() = default;

    /**
     *
     * @param robot The current robot instance
     * @param target_pos The target robot pos
     * @return the voltage to apply to each motor
     */
    virtual void applyController(AbstractRobot& robot, const Position& target_pos) = 0;

    virtual void reset_to(const AbstractRobot& robot, const Position& position) = 0;
};



#endif //ABSTRACTCONTROLLER_H
