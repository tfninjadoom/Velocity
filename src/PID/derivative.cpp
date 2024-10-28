// member.cpp
#include "PID/derivative.hpp"

DerivativeBase::DerivativeBase(const double& value) :
    m_value(value)
{}

double DerivativeBase::calculate() const {
    return 2.0 * m_value;
}

DerivativeDerived::DerivativeDerived(const double& value) :
    DerivativeBase::DerivativeBase(value)
{}
