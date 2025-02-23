//
// Created by fogoz on 17/02/2025.
//

#ifndef TEENSYCODE2_0_PIDCONTROLLERDETERMINISTICRAMP_H
#define TEENSYCODE2_0_PIDCONTROLLERDETERMINISTICRAMP_H
#include <memory>
#include "AbstractController.h"
#include "utils/Position.h"
#include "utils/PID.h"
#include "utils/Ramp.h"

class PIDControllerDeterministicRamp : public AbstractController{
    std::shared_ptr<PID> distancePID;
    std::shared_ptr<PID> anglePID;
    std::shared_ptr<Ramp> rampDistance;

    double distance_target = 0.0f;
    double angle_target = 0.0f;

    Position pos;


public:
    PIDControllerDeterministicRamp(std::shared_ptr<PID> distancePID, std::shared_ptr<PID> anglePID, std::shared_ptr<Ramp> rampDistance);

    void applyController(AbstractRobot &robot, const Position &target_pos) override;

    void reset_to(AbstractRobot& robot, const Position &position) override;

    void setTargetPos(AbstractRobot &robot, const Position &position) override;
};


#endif //TEENSYCODE2_0_PIDCONTROLLERDETERMINISTICRAMP_H
