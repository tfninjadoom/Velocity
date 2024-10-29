#include "neutral_mech.hpp"

namespace Neutral {

//MOVE TO MAIN AND CHANGE PORTS
pros::Rotation neutralRotation(0); 
pros::Motor    neutralMotor(0);

void initPID(double Kp, double Ki, double Kd, bool resetSensor) {
    if (resetSensor) { neutralRotation.reset_position(); }
    
    //PID_AntiWindup neutralPID(1,0,1,1);
    SimplePID neutralPID(1,0,1);

    pros::Task neutralTask(
        [&]()->void {
            double currentState { (double)neutralRotation.get_position() };
            double output { neutralPID.calculate(Neutral::target, currentState) };
            neutralMotor.move_voltage(output);
            pros::delay(10);
        }
    );
}

} //namespace Neutral