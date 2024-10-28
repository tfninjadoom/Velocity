// member.cpp
#include "PID/integral.hpp"

IntegralBase::IntegralBase(const double& value) :
    m_value(value)
{}

double IntegralBase::calculate() const {
    return 2.0 * m_value;
}

IntegralDerived::IntegralDerived(const double& value) :
    IntegralBase::IntegralBase(value)
{}
