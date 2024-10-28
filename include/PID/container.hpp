// container.hpp
#pragma once
#include <memory>
#include "PID/member.hpp"

class ContainerBase {

public:
    virtual void doStuff() = 0;

protected:
    double m_value;

    ContainerBase(double value);

    std::unique_ptr<MemberBase> m_member;

};

class ContainerDerived : ContainerBase {

public:
    ContainerDerived(double value);
    void doStuff();

};

//godbolt link: https://gcc.godbolt.org/z/KdYToYdET 