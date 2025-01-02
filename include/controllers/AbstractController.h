//
// Created by fogoz on 30/12/2024.
//

#ifndef ABSTRACTCONTROLLER_H
#define ABSTRACTCONTROLLER_H
#include <tuple>

class AbstractController {
public:
    /**
     *
     * @param robot The current robot instance
     * @param target_pos The target robot pos
     * @return the voltage to apply to each motor
     */
    //virtual std::tuple<double, double> applyController(const AbstractRobot& robot, const Position& target_pos);
};



#endif //ABSTRACTCONTROLLER_H
