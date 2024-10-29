// container.cpp
#include "PID/container.hpp"



//---------------------------------- SimplePID ------------------------------------//
SimplePID::SimplePID(double Kp, double Ki, double Kd) :
    m_Kp(Kp), m_Ki(Ki), m_Kd(Kd), m_error() //nullptr
{}

double SimplePID::calculate(double target, double currentState) {
    m_pastError = m_error;
    m_error = target - currentState;

    return (m_Kp * (m_error) + m_Ki * (m_integral += m_pastError) + m_Kd * (m_error - m_pastError));
}