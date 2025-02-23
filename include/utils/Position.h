//
// Created by fogoz on 28/12/2024.
//

#ifndef POSITION_H
#define POSITION_H

#include <Printable.h>
#include "Print.h"
#include <cmath>
#define WARP_ANGLE_DEG(angle) fmod(fmod(angle + 180, 360) - 360, 360) + 180
#define WARP_ANGLE(angle) fmod(fmod(angle + M_PI, 2*M_PI) - 2*M_PI, 2*M_PI) + M_PI
class Position : public Printable{
private:
    double x;
    double y;
    double theta;


public:

    Position(double x=0.0f, double y=0.0f, double theta=0.0f)
        : x(x),
          y(y),
          theta(WARP_ANGLE(theta)) {
    }
    size_t printTo(Print &p) const override {
        return p.printf("Position (%f, %f, %f)", x, y, theta);
    }

    Position& operator+=(const Position& rhs) {
        x += rhs.x;
        y += rhs.y;
        theta += rhs.theta;
        theta = WARP_ANGLE(theta);
        return *this;
    }
    Position operator+(const Position& rhs) const {
        return {x + rhs.x, y + rhs.y, theta + rhs.theta};
    }
    Position& operator-=(const Position& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        theta -= rhs.theta;
        theta = WARP_ANGLE(theta);
        return *this;
    }
    Position operator-(const Position& rhs) const {
        return {x - rhs.x, y - rhs.y, theta - rhs.theta};
    }

    double getDistance() const {
        return sqrt(pow(x, 2) + pow(y, 2));
    }

    double getDistanceAngle() const {
        return atan2(y, x);
    }

    double getDistanceAngleDeg() const {
        return getDistanceAngle() * 180 / M_PI;
    }

    double getX() const {
        return x;
    }

    double getY() const {
        return y;
    }

    double getAngle() const {
        return theta;
    }

    double getAngleDeg() const {
        return getAngle() * 180 / M_PI;
    }

    bool operator!=(const Position& position) const{
        return x==position.x && y == position.y && theta == position.theta;
    }
};
#endif //POSITION_H
