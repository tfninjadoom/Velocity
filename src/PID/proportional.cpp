// member.cpp
#include "PID/proportional.hpp"

ProportionalBase::ProportionalBase(const double& value) :
    m_value(value)
{}

double ProportionalBase::calculate() const {
    return 2.0 * m_value;
}

ProportionalDerived::ProportionalDerived(const double& value) :
    ProportionalBase::ProportionalBase(value)
{}
