// member.hpp
#pragma once

class MemberBase {

public:
    virtual double returnVal() const = 0;

protected:
    const double& m_value;

    MemberBase(const double& value);

};

class MemberDerived : public MemberBase {

public:
    MemberDerived(const double& value);
    double returnVal() const override;

};

// godbolt link: https://gcc.godbolt.org/z/KdYToYdET 