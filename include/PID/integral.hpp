// member.hpp
#pragma once

class IntegralBase {

public:
    virtual double calculate() const;

protected:
    const double& m_value;

    IntegralBase(const double& value);

};

class IntegralDerived : public IntegralBase {

public:
    IntegralDerived(const double& value);
    double calculate() const override;

};