// container.hpp
#pragma once
#include <memory>
#include "PID/proportional.hpp"
#include "PID/integral.hpp"
#include "PID/derivative.hpp"



//---------------------------------- SimplePID ------------------------------------//
class SimplePID {

public:
    SimplePID(double Kp, double Ki, double Kd);
    double calculate(double target, double currentState);

private:
    double m_Kp, m_Ki, m_Kd;
    double m_error = 0;
    double m_pastError = 0;
    double m_integral = 0;

};