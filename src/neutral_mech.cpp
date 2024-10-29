#include "neutral_mech.hpp"

namespace Neutral {

//MOVE TO MAIN AND CHANGE PORTS
pros::Rotation neutralRotation(0); 
pros::Motor    neutralMotor(0);

void initPID(double Kp, double Ki, double Kd, bool resetSensor) {
    if (resetSensor) { neutralRotation.set_position(0); }
    
    double min = -9000, max = 9000; //75% of motor speed
    SimplePID neutralPID(1,0,1);
    //PID_AntiWindup neutralPID(1,0,1,1);

    pros::Task neutralTask(
        [&]()->void {
            double currentState { (double)neutralRotation.get_position() };
            double output { neutralPID.calculate(Neutral::target, currentState) };

            // clamps value to min & max
            const double temp = output < min ? min : output;
            output = temp > max ? max : output;

            neutralMotor.move_voltage(output); //input range of +-12000mV
            pros::delay(10);
        }
    );
}

} //namespace Neutral