//
// Created by fogoz on 28/12/2024.
//

#ifndef POSITION_H
#define POSITION_H
class Position : public Printable{
private:
    double x;
    double y;
    double theta;


public:

    Position(double x=0.0f, double y=0.0f, double theta=0.0f)
        : x(x),
          y(y),
          theta(theta) {
    }
    size_t printTo(Print &p) const override {
        return Serial.printf("Position (%f, %f, %f)", x, y, theta);
    }

    Position& operator+=(const Position& rhs) {
        x += rhs.x;
        y += rhs.y;
        theta += rhs.theta;
        return *this;
    }
    Position operator+(const Position& rhs) const {
        return {x + rhs.x, y + rhs.y, theta + rhs.theta};
    }
    Position& operator-=(const Position& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        theta -= rhs.theta;
        return *this;
    }
    Position operator-(const Position& rhs) const {
        return {x - rhs.x, y - rhs.y, theta - rhs.theta};
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
};
#endif //POSITION_H
