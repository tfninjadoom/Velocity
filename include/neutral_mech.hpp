#include "main.h"
#include "PID/container.hpp"


inline namespace Neutral { 

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

Neutral::State target = Neutral::rest;

void initPID(double Kp, double Ki, double Kd, bool resetSensor=false);

} //namespace Neutral