#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/timer.hpp"
#include "neutral_mech.hpp"
#include "pros/misc.h"
#include "pros/optical.hpp"

// Shift Key Macro
//#define SHIFT controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)

// Competition Initialize?
bool compInit = false;

// forward-declarations
void turn90();
void redSoloAWP();
void blueSoloAWP();
void oldRedSoloAWP();
void PDtune();

// movement PID constants
double vertKp = 0;
double vertKd = 0;
double horizKp = 0;
double horizKd = 0;
double thetaKp = 0;
double thetaKd = 0;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({19,20}, pros::MotorGearset::green); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({16,17}, pros::MotorGearset::green); // right motor group - ports 6, 7, 9 (reversed)

// sensor ports
pros::Imu imu(4);
pros::Optical optical(21);

// drive ports
pros::Motor leftFront55(11);
pros::Motor leftFront11(12);
pros::Motor rightFront55(13);
pros::Motor rightFront11(14); 
pros::Motor leftBack55(15); 
pros::Motor leftBack11(16);
pros::Motor rightBack55(17);
pros::Motor rightBack11(18);

// motor ports
pros::Motor intake(8);

// solenoid ports
pros::adi::DigitalOut clamp ('A');
pros::adi::DigitalOut didler ('B');
pros::adi::DigitalOut hang ('C');

// drive control type
void holonomicDrive(double angular, double vertical, double horizontal)
{
	// leftTop.move(vertical+angular+horizontal);
	// leftBottom.move(vertical+angular-horizontal);
	// rightBottom.move(vertical-angular+horizontal);
	// rightTop.move(vertical-angular-horizontal);

    leftFront55.move( -(angular+vertical+horizontal) );
    leftFront11.move(  (angular+vertical+horizontal) );
    rightFront55.move(-(angular-vertical+horizontal) );
    rightFront11.move( (angular-vertical+horizontal) );
    leftBack55.move(  -(angular+vertical-horizontal) );
    leftBack11.move(   (angular+vertical-horizontal) );
    rightBack55.move( -(angular-vertical-horizontal) );
    rightBack11.move(  (angular-vertical-horizontal) );

}

// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-21);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(15);
pros::Rotation verticalEnc2(-18);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 1);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 1);
lemlib::TrackingWheel vertical2(&verticalEnc2, lemlib::Omniwheel::NEW_275, -1);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              15, // 15 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              333, // drivetrain rpm is 200
                              5 // horizontal drift: 2 for all-omni tank drive, 8 for omni-traction tank drive
);

// vertical motion controller
lemlib::ControllerSettings linearController(2, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            8, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3,    // joystick deadband out of 127
                                  10,   // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void telemetry() {
    while (true) {
        // print robot location to the brain screen
        pros::lcd::print(0, "X: %.2f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %.2f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta); // heading

        // printing PD constants
        pros::lcd::print(4, "thetaKp: %.2f", thetaKp); // thetaKp
        pros::lcd::print(5, "thetaKd: %.2f", thetaKd); // thetaKd

        // log position telemetry
        // lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
        // delay to save resources
        pros::delay(50);

         
    }
}

void colorSorterInit() {
    bool blue = false;
    pros::Task colorSorter { 
        [blue] () -> void {
            // hue values are from 0 to 359.999
            double redLow = 320.;
            double redHigh = 100.; 
            double blueLow = 210.;
            double blueHigh = 280.;
            
            auto stallConveyor = []() -> void {
                pros::delay(50);
                double prevVoltage = intake.get_voltage();
                intake.move(0);
                pros::delay(100);
                intake.move_voltage(prevVoltage);
            };

            while (true) {
                if (blue) {
                    if (optical.get_hue() > blueLow && optical.get_hue() < blueHigh) { stallConveyor(); }
                } else {
                    if (optical.get_hue() > redLow || optical.get_hue() < redHigh)   { stallConveyor(); }
                }
                pros::delay(10);
            }
        }
    };
}


void initialize() {
    imu.reset();
    chassis.calibrate(); // calibrate sensors
    if (!pros::lcd::is_initialized()) pros::lcd::initialize();
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging distance(x,y,chassis.getPose().x,chassis.getPose().y)>2.5
    pros::Task screenTask(telemetry);
    chassis.setPose(0,0,0);
    Neutral::initPID(Neutral::Kp, Neutral::Ki, Neutral::Kd);

    colorSorterInit();

}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() { compInit = true; }

// get a path used for pure pursuit
// this needs to be put outside a function

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 *  chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90ยบ. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
 */

void autonomous() {
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

}

void opcontrol() {
    bool tuningPID = false;

    // loop to continuously updatePD motors
    while (true) {
        
        // robot state
        bool didlerUp = true;
        int intakeInUse = 0; // 0 : driver control, 1 : in use by neutralCancel Task

        // program state
		bool B_Pressed;

		double forward = controller.get_analog(ANALOG_LEFT_Y);
	  	double turn = controller.get_analog(ANALOG_RIGHT_X);
	  	double strafe = controller.get_analog(ANALOG_LEFT_X);
		holonomicDrive(turn, forward, strafe);

        //----------------------DRIVER CONTROLS----------------------//

        // Intake / Conveyor 
        if (intakeInUse == 0) {
            if      (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) 
            {
                intake.move(127);
            }
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) 
            {
                intake.move(-127);
            }
            else {
                intake.move(0);
            }
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) || 
                   controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) 
        {
            intakeInUse = 0;
        }

        // Clamp
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
        {
            clamp.set_value(true);
        }
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
        {
            clamp.set_value(false);
        }

        // Neutral Load Ring 1
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
        {
            Neutral::target = ring1;
        }

        // Neutral Score
        // Neutral Rest on Second Press
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)
           && Neutral::target != score ) 
        {
		    Neutral::target = score;
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)
            && Neutral::target == score ) 
        {
			Neutral::target = rest;
        }

        // Neutral Descore
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) ) 
        {
			Neutral::target = descore;
        }

        // Neutral Clear
        // Deploys HANG
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) 
        {
            Neutral::target = clear;
        }


        // Neutral Cancel
        // Removes ring from neutral by spinning conveyor in reverse
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            intakeInUse = 1;
            pros::Task neutralCancel (
                [intakeInUse]()->void {
                    // Task can be canceled by pressing R1/R2
                    if (intakeInUse == 1) intake.move(-127);
                    if (intakeInUse == 1) pros::delay(750);
                    if (intakeInUse == 1) intake.move(0);
                }
            );
        }

        // Didler Toggle
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) 
        {
            didler.set_value(!didlerUp);
            didlerUp = !didlerUp;
        }

        /** Neutral Emergency Stick Control
         * 
         *      Turns on when you start holding DOWN
         *      Joystick functions while DOWN is held
         *      Control returned to task after pressing Shift+DOWN
         */
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) 
        {
            // only works when holding DOWN
            neutralMotor.move( controller.get_analog(ANALOG_LEFT_Y) ); 
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) 
        {
            Neutral::emergencyControl = true; // taking neutral controls from task
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) == false) 
        {
            Neutral::emergencyControl = false; // returning neutral controls to task
        }

        //-----------------------UTILITY BUTTONS (PROG)------------------------//

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) && !compInit) // DOES NOT WORK WHEN CONNECTED TO COMPETITION CONTROL
		{
            autonomous();
        }

        // if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && !tuningPID) {
		// 	redSoloAWP();
        // }

        // if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && !tuningPID) {
		// 	blueSoloAWP();
        // }

        // if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && !tuningPID) {
		// 	tuningPID = true;
        // } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && tuningPID) {
		// 	tuningPID = false;
        // }

        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) && tuningPID) {
        //     thetaKp += 0.05;
        // }

        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) && tuningPID && B_Pressed ) {
        //     thetaKp -= 0.05;
        // }

        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        //     imu.set_heading(0);
        //     chassis.setPose(0,0,0);
        // }

		pros::delay(20);                             
	}
}


//------------------------------------------ AUTONS ------------------------------------------------------//

// UTILS

double updatePD(double Kp, double Kd, double error, double prevError) {
    return Kp*error + Kd*(error - prevError);
}

double checkAngle(double angError) {
    if (angError > 180) angError -= 360;
    else if (angError < - 180) angError += 360;
    return angError;
}

void moveTo(const double& x, const double& y, const double& theta, const int& timeout) {

    double verticalError = y - chassis.getPose().y;
    double verticalPrevError = verticalError;
    double horizontalError = x - chassis.getPose().x;
    double horizontalPrevError = horizontalError;
    double thetaError = checkAngle(theta - imu.get_heading());
    double thetaPrevError = thetaError;
    double heading = imu.get_heading();

    lemlib::Timer timer(timeout);
    double verticalMtr, horizontalMtr, thetaMtr = 0;

    while(!timer.isDone()) {
        heading = imu.get_heading();
	    if (heading > 180)  heading -= 360;
	    if (heading < -180) heading += 360;
	    
        verticalError = y - chassis.getPose().y;
        verticalMtr = updatePD(vertKp, vertKd, verticalError, verticalPrevError);

        horizontalError = x - chassis.getPose().x;
        horizontalMtr = updatePD(horizKp, horizKd, horizontalError, horizontalPrevError);

        thetaError = checkAngle(theta-imu.get_heading());
        thetaMtr = updatePD(thetaKp, thetaKd, thetaError, thetaPrevError);

        double ADverticalMtr = verticalMtr * cos(heading * M_PI / 180) + horizontalMtr * sin(heading * M_PI / 180); // Adjust based off of heading
        double ADhorizontalMtr = -verticalMtr * sin(heading * M_PI / 180) + horizontalMtr * cos(heading * M_PI / 180);

        holonomicDrive(thetaMtr,ADverticalMtr,ADhorizontalMtr);

        verticalPrevError = verticalError;
        horizontalPrevError = horizontalError;
        thetaPrevError = thetaError;
        // pros::lcd::print(5," Horizontal %.2f",horizontalMtr);
        // pros::lcd::print(6,"vertical %.2f",verticalMtr);
        // pros::lcd::print(7,"%.2f",heading);
        // pros::lcd::print(4,"error %.2f", thetaMtr);

        if (fabs(verticalError) < 1 &&
            fabs(horizontalError) < 1 &&
            fabs(thetaError) < 1)
        {
            break;
        }

        pros::delay(20);
    }

    holonomicDrive(0,0,0);
}

// PATHS

//empty