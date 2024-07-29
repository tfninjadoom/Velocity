#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/timer.hpp"


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({19,20},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({16,17}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(4);
pros::Motor leftTop(19);
pros::Motor leftBottom(16); 
pros::Motor rightTop(-20);
pros::Motor rightBottom(-17);
pros::ADIDigitalOut piston ('A');

pros::Motor intake1(-14);
pros::Motor intake2(-12);
pros::Motor    intake(8);

void arcadecontrolx(double angular, double vertical, double horizontal)
{
	leftTop.move(vertical+angular+horizontal);
	leftBottom.move(vertical+angular-horizontal);
	rightBottom.move(vertical-angular+horizontal);
	rightTop.move(vertical-angular-horizontal);

}
// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-21);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
pros::Rotation verticalEn2c(18);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 3.5);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 6.75);
lemlib::TrackingWheel vertical2(&verticalEn2c, lemlib::Omniwheel::NEW_275, -6.75);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
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
                            &vertical2, // vertical tracking wheel 2, set to nullptr as we don't have a second one
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
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
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
void odom(){
    while (true) {
        // print robot location to the brain screen
        pros::lcd::print(0, "X: %.2f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %.2f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %.2f", imu.get_heading()); // heading
        // log position telemetry
        // lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
        // delay to save resources
        pros::delay(50);
    }
}
void initialize() {
    chassis.calibrate(); // calibrate sensors
    pros::delay(2000);
    if (!pros::lcd::is_initialized()) pros::lcd::initialize();
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging distance(x,y,chassis.getPose().x,chassis.getPose().y)>2.5
    pros::Task screenTask(odom);
}
double update(double kp, double kd, double error, double prevError ) {

    return kp*error+kd*(error-prevError);

}
double checkAngle(double aerror) {
    if (aerror > 180) aerror -= 360;
    else if (aerror < - 180) aerror += 360;
    return aerror;
}

double verticalkp = 5.6;
double verticalkd = 3.2;
double horizontalkp=5.6;
double horizontalkd=2.4;
double thetakp=2;
double thetakd=4;

void moveforward(double x, double y, double theta, int timeout) {

    double verticalError = y-chassis.getPose().y;
    double verticalPrevError = verticalError;
    double horizontalError = x-chassis.getPose().x;
    double horizontalprevError = horizontalError;
    double thetaError = checkAngle(theta-imu.get_heading());
    double thetaprevError = thetaError;
    double heading = imu.get_heading();
    lemlib::Timer timer(timeout);
    double verticalMtr, horizontalMtr, thetaMtr
    while(!timer.isDone()) {

	if (heading > 180) heading -= 360;
	if (heading < - 180) heading += 360;
	    
        verticalError = y-chassis.getPose().y;
        verticalMtr = update(verticalkp, verticalkd, verticalError, verticalPrevError);

        horizontalError = x-chassis.getPose().x;
        horizontalMtr = update(horizontalkp, horizontalkd, horizontalError, horizontalprevError);

        thetaError = checkAngle(theta-imu.get_heading());
        thetaMtr = update(thetakp, thetakd, thetaError, thetaprevError);

        double ADverticalMtr = verticalMtr * cos(heading * M_PI / 180) + horizontalMtr * sin(heading * M_PI / 180); // Adjust based off of heading
        double ADhorizontalMtr = -verticalMtr * sin(heading * M_PI / 180) + horizontalMtr * cos(heading * M_PI / 180);

        arcadecontrolx(thetaMtr,ADverticalMtr,ADhorizontalMtr);

        verticalPrevError=verticalError;
        horizontalprevError=horizontalError;
        thetaprevError=thetaError;
        pros::lcd::print(5," Horizontal %.2f",ADhorizontalMtr);
        pros::lcd::print(6,"vertical %.2f",ADverticalMtr);
        pros::lcd::print(7,"%.2f",thetaError);

        if (fabs(verticalError) < 0.5 &&
            fabs(horizontalError) < 0.5 &&
            fabs(thetaError) < 0.5){
            break;
        }
        pros::delay(20);
    }
    arcadecontrolx(0,0,0);
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

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

void conveyor(int speed){
    intake1.move(-speed);
    intake2.move(speed);
}

void autonomous() {
    piston.set_value(1);
    moveforward(0,-18,0,1800);
    piston.set_value(0);
    pros::delay(200);
    conveyor(127);
    pros::delay(800);
    conveyor(0);
    moveforward(10,-23,90,1500);
    piston.set_value(1);
    intake.move(-127); // intake
    conveyor(127);
    moveforward(17.5,-23,90,1500);
    pros::delay(500);
    conveyor(0);
    pros::delay(200);
    intake.move(127);
    piston.set_value(1);
    moveforward(0,0,135,3000);
    moveforward(24,48,90,3000);
    

    
}

void opcontrol() {
    // controller
    	pros::Controller master(pros::E_CONTROLLER_MASTER);

    // loop to continuously update motors
    while (true) {
		bool r1pressed;
      	bool r2pressed;
		bool l1pressed;
      	bool l2pressed;
		bool apressed;

		double left = master.get_analog(ANALOG_LEFT_Y);
	  	double right = master.get_analog(ANALOG_RIGHT_X);
	  	double rightx = master.get_analog(ANALOG_LEFT_X);
		arcadecontrolx(right, left, rightx);
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
           intake1.move(127);
		   intake2.move(-127);
		    intake.move(127);
      }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake1.move(-127);
		   intake2.move(127);
		    intake.move(-127);

        }  else {
			intake1.move(0);
		   intake2.move(0);
		    intake.move(0);
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
           piston.set_value(true);
        }
      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			piston.set_value(false);

        }          
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			intake1.move(0);
		   intake2.move(0);
		    intake.move(0);

        }   
		pros::delay(20);                             
	}
}


