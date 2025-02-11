//
// Created by fogoz on 03/01/2025.
//

#ifndef TEENSYCODE2_0_PIDCONTROLLER_H
#define TEENSYCODE2_0_PIDCONTROLLER_H


#include <memory>
#include "AbstractController.h"
#include "utils/Position.h"
#include "utils/PID.h"
#include "utils/QuadRamp.h"

class PIDController : public AbstractController {
    std::shared_ptr<PID> distancePID;
    std::shared_ptr<PID> anglePID;
    std::shared_ptr<QuadRamp> quadRampDistance;
    std::shared_ptr<QuadRamp> quadRampAngle;

    double distance_target = 0.0f;
    double angle_target = 0.0f;

public:
    PIDController(std::shared_ptr<PID> distancePID, std::shared_ptr<PID> anglePID, std::shared_ptr<QuadRamp> rampDistance, std::shared_ptr<QuadRamp> rampAngle);

    void applyController(AbstractRobot &robot, const Position &target_pos) override;

    void reset_to(AbstractRobot& robot, const Position &position) override;
};


#endif //TEENSYCODE2_0_PIDCONTROLLER_H
