// container.hpp
#pragma once
#include <memory>
#include "PID/proportional.hpp"
#include "PID/integral.hpp"
#include "PID/derivative.hpp"

class PID_Base {

public:
    PID_Base(double Kp, double Ki, double Kd);
    virtual double calculate();

protected:
    double m_Kp;
    double m_Ki;
    double m_Kd;
    double m_member;

    std::unique_ptr<ProportionalBase>   m_P;
    std::unique_ptr<IntegralBase>       m_I;
    std::unique_ptr<DerivativeBase>     m_D;

};

class PID_Derived : PID_Base {

public:
    PID_Derived(double Kp, double Ki, double Kd, double value);
    double calculate();

};