#ifndef NATIVE
#include <Arduino.h>
#else
#include "iostream"
#endif

#include "utils/Matrix.h"
#include "vector"
#include "motors/DirPWMMotor.h"
#include "encoders/QuadEncoderImpl.h"
#include "robot/HomogeneousMatrixRobotImpl.h"
AbstractRobot* robot;
void setup() {
// write your initialization code here
}

void loop() {
// write your code here
}

#ifdef NATIVE
int main(){
    setup();
    Matrix<double> matrix = Matrix<double>::getEye(4) * 4 + 1;
    std::vector<double> vector = (matrix * Matrix<double>::getEye(4)).getVector();
    for(auto a : vector){
        std::cout << a << ", ";
    }
    std::cout << std::endl;
    while(true)
        loop();
    return 0;
}



#endif