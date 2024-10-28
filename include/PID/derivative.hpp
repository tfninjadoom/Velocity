// member.hpp
#pragma once

class DerivativeBase {

public:
    virtual double calculate() const;

protected:
    const double& m_value;

    DerivativeBase(const double& value);

};

class DerivativeDerived : public DerivativeBase {

public:
    DerivativeDerived(const double& value);
    double calculate() const override;

};