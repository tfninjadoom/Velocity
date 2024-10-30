// container.cpp
#include "PID/container.hpp"

PID_Base::PID_Base(double Kp, double Ki, double Kd) :
    m_Kp(Kp), m_Ki(Ki), m_Kd(Kd), m_error() //nullptr
{}

double PID_Base::calculate(double target, double currentState) {
    
    return (m_Kp * m_P->calculate() + m_Ki * m_I->calculate() + m_Kd * m_D->calculate());
}

PID_Derived::PID_Derived(double Kp, double Ki, double Kd, double value) :
    PID_Base::PID_Base(Kp, Ki, Kd)
{
    m_P = std::make_unique<ProportionalDerived>(value);
    m_I = std::make_unique<IntegralDerived>(value);
    m_D = std::make_unique<DerivativeDerived>(value);
}

double PID_Derived::calculate(double target, double currentState) {
    // do stuff with m_value and m_member
    return 0.;
}

PID_AntiWindup::PID_AntiWindup(double Kp, double Ki, double Kd, double maxWindup) :
    PID_Base::PID_Base(Kp, Ki, Kd), m_max(maxWindup)
{
    return;
}

double PID_AntiWindup::calculate(double target, double currentState) {
    return 0.;
}

//---------------------------------- SimplePID ------------------------------------//
SimplePID::SimplePID(double Kp, double Ki, double Kd) :
    m_Kp(Kp), m_Ki(Ki), m_Kd(Kd), m_error() //nullptr
{}

double SimplePID::calculate(double target, double currentState) {
    m_pastError = m_error;
    m_error = target - currentState;

    return (m_Kp * (m_error) + m_Ki * (m_integral += m_pastError) + m_Kd * (m_error - m_pastError));
}