// container.cpp
#include "PID/container.hpp"

PID_Base::PID_Base(double Kp, double Ki, double Kd) :
    m_Kp(Kp), m_Ki(Ki), m_Kd(Kd), m_member() //nullptr
{}

double PID_Base::calculate() {
    return (m_Kp * m_P->calculate() + m_Ki * m_I->calculate() + m_Kd * m_D->calculate());
}

PID_Derived::PID_Derived(double Kp, double Ki, double Kd, double value) :
    PID_Base::PID_Base(Kp, Ki, Kd)
{
    m_P = std::make_unique<ProportionalDerived>(value);
    m_I = std::make_unique<IntegralDerived>(value);
    m_D = std::make_unique<DerivativeDerived>(value);
}

double PID_Derived::calculate() {
    // do stuff with m_value and m_member
}