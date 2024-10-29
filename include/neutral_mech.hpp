#include "main.h"
#include "PID/container.hpp"


inline namespace Neutral { 

enum State {
    rest = 0,
    ring1 = 1,
    ring2 = 2,
    clear = 3,
    score = 4,
    descore = 5
};

} //inline namespace Neutral

namespace Neutral {

Neutral::State target = Neutral::rest;

void initPID(double Kp, double Ki, double Kd, bool resetSensor=false);

} //namespace Neutral