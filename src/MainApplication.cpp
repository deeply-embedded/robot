/*
 * MainApplication.cpp

 *
 *  Created on: 24.06.2016
 *      Author: abn672
 */
#include "periphery/actor/Servo.h"
#include "RobotLeg.hpp"
#include <thread>
#include <chrono>

int main(){

RSL::Servo servo1(RSL::GPIOPin::P9_42);
RSL::Servo servo2(RSL::GPIOPin::P9_22);

RobotLeg leg1(&servo1, &servo2, 200, 0, 90, 0, 90);

while(1) {
	leg1.move();

	std::this_thread::sleep_for(std::chrono::seconds{ 1 });

}


}



