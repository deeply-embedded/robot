/*
 * RobotLeg.cpp
 *
 *  Created on: 24.06.2016
 *      Author: abn672
 */

/*
definition of min and max:
there are not abolute ranges, but the used ranges!
- hip servo (vertical revolution axis)
  * min: leg in vorward position (stepping down)
  * max: leg in aft position (lifting leg)
- knee servo (horizontal revolution axis)
  * min: leg straighten / robot lifted
  * max: leg bent / leg lifted / robot down
*/

#include "periphery/actor/Servo.h"



class RobotLeg {
public:
  RobotLeg(RSL::Servo* hip, RSL::Servo* knee, int cycleSteps, int hip_min, int hip_max, int knee_min, int knee_max) {
    this->hip = hip;
    this->knee = knee;
    this->state = 1;
    this->hip_min = hip_min;
    this->hip_max = hip_max;
    this->knee_min = knee_min;
    this->knee_max = knee_max;

    moveSteps = cycleSteps / 4;
    fastSteps = moveSteps / 3;

    moveTravel = (hip_max - hip_min);
    riseTravel = (knee_max - knee_min);

    hip->enableServo();
    knee->enableServo();

    setToStateInitPosition(1);


  }

  void move() {
    switch (internalState) {
      case 1:
        if (step < moveSteps*3) {
          hip->setPosition(degToServo(hip_min + moveTravel/moveSteps));
        } else {
          setToStateInitPosition(4);
          state = 4;
          step = 0;
          internalState = 2;
        }
        break;
      case 2:
        if (step < fastSteps) {
          knee->setPosition(degToServo(knee_min + riseTravel/fastSteps));
        } else {
          state = 4;
          step = 0;
          internalState = 3;
        }
        break;
      case 3:
        if (step < fastSteps) {
          hip->setPosition(degToServo(hip_max - moveTravel/fastSteps));
        } else {
          state = 4;
          step = 0;
          internalState = 3;
        }
        break;
      case 4:
        if (step < fastSteps) {
          knee->setPosition(degToServo(knee_max - riseTravel/fastSteps));
        } else {
          state = 1;
          step = 0;
          internalState = 1;
        }
        break;
      default:
        // ERROR
    	  ;
    }
  }

  void setToStateInitPosition(int state) {
    step = 0;
    this->state = state;
    switch (state) {
      case 1:
        hip->setPosition(degToServo(hip_min));
        knee->setPosition(degToServo(knee_max));
        internalState = 1;
        step = 0;
        break;
      case 2:
        hip->setPosition(degToServo(hip_min + moveSteps));
        knee->setPosition(degToServo(knee_max));
        internalState = 1;
        step = moveSteps * 1;
        break;
      case 3:
        hip->setPosition(degToServo(hip_min + moveSteps*2));
        knee->setPosition(degToServo(knee_max));
        internalState = 1;
        step = moveSteps * 2;
        break;
      case 4:
        hip->setPosition(degToServo(hip_max));
        knee->setPosition(degToServo(knee_max));
        internalState = 2;
        step = 0;
        break;
      default:
        // ERROR
    	  ;
    }
  }

  float degToServo(int deg) {
    return deg / 180;
  }

  void setPhase(int state) { // state 1, 2, 3, 4 according to docs
    if ((state >= 1) && (state <= 4)) {
      this->state = state;
      step = 0;

      setToStateInitPosition(state);
    }
  }

private:
  RSL::Servo* hip;
  RSL::Servo* knee;
  int hip_min, hip_max, knee_min, knee_max;
  int cycleSteps, moveSteps, fastSteps;
  int moveTravel, riseTravel;
  int state;
  int internalState;
  int step;

};



