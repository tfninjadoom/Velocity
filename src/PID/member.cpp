// member.cpp
#include "PID/member.hpp"

MemberBase::MemberBase(const double& value) :
    m_value(value)
{}

MemberDerived::MemberDerived(const double& value) :
    MemberBase::MemberBase(value)
{}

double MemberDerived::returnVal() const {
    return 2.0 * m_value;
}

// godbolt link: https://gcc.godbolt.org/z/KdYToYdET 