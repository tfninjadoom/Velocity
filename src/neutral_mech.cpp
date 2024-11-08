#include "neutral_mech.hpp"

namespace Neutral {

//MOVE TO MAIN AND CHANGE PORTS
pros::Rotation neutralRotation { 0}; 
pros::Motor    neutralMotor    {-0};

void initPID(double Kp, double Ki, double Kd, bool resetSensor) {
    if (resetSensor) { neutralRotation.set_position(0); }

    SimplePID neutralPID(
        // PID Constants
        Kp,
        Ki,
        Kd
    );
    const double min = Neutral::min, max = Neutral::max;

    pros::Task neutralTask(
        [&neutralPID, min, max]()->void {
            while (true) { 
                if (paused || Neutral::emergencyControl) { pros::delay(30); continue; }

                double currentState { (double)neutralRotation.get_position() };
                double output { neutralPID.calculate(Neutral::target, currentState) };

                // clamps value to min & max
                const double temp = output < min ? min : output;
                output = temp > max ? max : output;

                neutralMotor.move_voltage(output); //input range of +-12000mV
                pros::delay(20);
            }
        }
    );
}

} //namespace Neutral