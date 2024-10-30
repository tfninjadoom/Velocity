// container.hpp
#pragma once
#include <memory>
#include "PID/proportional.hpp"
#include "PID/integral.hpp"
#include "PID/derivative.hpp"

class PID_Base {

public:
    PID_Base(double Kp, double Ki, double Kd);

    virtual double calculate(double target, double currentState);

protected:
    double m_Kp, m_Ki, m_Kd;
    double m_error;

    std::unique_ptr<ProportionalBase>   m_P;
    std::unique_ptr<IntegralBase>       m_I;
    std::unique_ptr<DerivativeBase>     m_D;

};

class PID_Derived : PID_Base {

public:
    PID_Derived(double Kp, double Ki, double Kd, double value);

    double calculate(double target, double currentState) override;

};

class PID_AntiWindup : PID_Base {

public:
    PID_AntiWindup(double Kp, double Ki, double Kd, double maxWindup);

    double calculate(double target, double currentState) override;

private:
    double m_max;

};

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