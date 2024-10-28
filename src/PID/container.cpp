// container.cpp
#include "PID/container.hpp"

ContainerBase::ContainerBase(double value) :
    m_value(value), m_member() //nullptr
{}

ContainerDerived::ContainerDerived(double value) :
    ContainerBase::ContainerBase(value)    
{
    m_member = std::make_unique<MemberDerived>(value);
}

void ContainerDerived::doStuff() {
    // do stuff with m_value and m_member
}

// godbolt link: https://gcc.godbolt.org/z/KdYToYdET 