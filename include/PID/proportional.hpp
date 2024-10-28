// member.hpp
#pragma once

class ProportionalBase {

public:
    virtual double calculate() const;

protected:
    const double& m_value;

    ProportionalBase(const double& value);

};

class ProportionalDerived : public ProportionalBase {

public:
    ProportionalDerived(const double& value);
    double calculate() const override;

};