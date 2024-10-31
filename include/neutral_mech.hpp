#pragma once
#include "main.h"
#include "PID/container.hpp"


inline namespace Neutral { 

// NEUTRAL MECH POSITIONS
enum State {
    rest = 0,
    ring1 = 2500,
    ring2 = 3000,
    clear = 6000,
    score = 13500,
    descore = 13500
};

} //inline namespace Neutral

namespace Neutral {

// PID CONSTANTS
const double
Kp = 0.66,
Ki = 0.0,
Kd = 1.0;

//limit to 75% of motor speed (range is +-12000)
const double min = -9000, max = 9000; 

extern pros::Rotation neutralRotation;
extern pros::Motor    neutralMotor;

Neutral::State target = Neutral::rest;
bool paused = false;

void initPID(double Kp, double Ki, double Kd, bool resetSensor=false);

} //namespace Neutral