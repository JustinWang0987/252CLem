#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"

//controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

//chasssis moto grou
pros::MotorGroup LeftMotors({14, 2}, pros::MotorGearset::green);
pros::MotorGroup RightMotors({-17, -10}, pros::MotorGearset::green);

//imu
pros::Imu imu(9);

//rotation sensor/tracking wheel initialization
pros::Rotation rotation_sensor_h(8);
pros::Rotation rotation_sensor_v(10);

lemlib::TrackingWheel vertical_trekingwheel(&rotation_sensor_v, 3.1875, 0.875);
lemlib::TrackingWheel horizontal_trekingwheel(&rotation_sensor_h, 3.1875 , -1.75);


//Drivetrain initialization
lemlib::Drivetrain drivetrain(&LeftMotors, // left motor group
                              &RightMotors, // right motor group
                              11.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              200, // drivetrain rpm is 200
                              2 // horizontal drift is 2 (for now)
);

//sensor initialization
lemlib::OdomSensors sensors(&vertical_trekingwheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_trekingwheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              2, // derivative gain (kD)
                                              0, // anti windup
                                              0.75, // small error range, in inches
                                              500, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              1000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              2 , // derivative gain (kD)
                                              0, // anti windup
                                              2, // small error range, in degrees
                                              500, // small error range timeout, in milliseconds
                                              5, // large error range, in degrees
                                              2000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(
									 3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(
								  3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);

float adjHeading = 0;

void update_screen(void* param){
    while (true) 
    {
        float adjHeading = imu.get_heading() + 3 * imu.get_rotation() / 360;
        if(adjHeading > 360)
        {
            adjHeading -= 360;
        }
        else if(adjHeading < 0)
        {
            adjHeading += 360;
        }
        else
        {
            // do nothing
        }
        
        // print robot location to the brain screen
        pros::lcd::print(0, "X: %.3f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %.3f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %.3f", chassis.getPose().theta); // heading
        pros::lcd::print(3, "H Rotation Sensor: %.3f", (rotation_sensor_h.get_position() * 3.25 * M_PI) / 36000);
        pros::lcd::print(4, "V Rotation Sensor 1: %.3f", (rotation_sensor_v.get_position() * 3.25 * M_PI) / 36000);
        pros::lcd::print(5, "Aj: %.3f", adjHeading);

        pros::delay(20);
    }
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();

    pros::Task Update_Screen(update_screen, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "update screen");

    chassis.calibrate();
    chassis.setPose(0, 0, 0);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
    float adjHeading = imu.get_heading() + 3 * imu.get_rotation() / 360;
    if(adjHeading > 360)
    {
        adjHeading -= 360;
    }
    else if(adjHeading < 0)
    {
        adjHeading += 360;
    }
    else
    {
        // do nothing
    }

    chassis.setPose(chassis.getPose().x, chassis.getPose().y, adjHeading);

    // chassis.moveToPose(-100, 125, -90, 7000);
    // chassis.turnToHeading(-45, 2000);
    // chassis.moveToPoint(0, 10, 7000, {.forwards=false});
    // chassis.moveToPose(0, 0, 0, 7000, {.forwards=false});

    chassis.moveToPose(0, 128, 0, 15000 );
    pros::delay(5000);
    chassis.moveToPose(0, 0, 0, 15000, {.forwards=false});
    
}
void autonomous2() {
    float adjHeading = imu.get_heading() + 3 * imu.get_rotation() / 360;
    if(adjHeading > 360)
    {
        adjHeading -= 360;
    }
    else if(adjHeading < 0)
    {
        adjHeading += 360;
    }
    else
    {
        // do nothing
    }

    chassis.setPose(chassis.getPose().x, chassis.getPose().y, adjHeading);

    chassis.moveToPose(0, 30, 0, 15000, {.maxSpeed=64});
    pros::delay(5000);
    chassis.moveToPose(0, 0, 0, 15000, {.forwards=false, .maxSpeed=64});
    
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
    while(1)
    {
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            autonomous();
        }
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
            autonomous2();
        }
        
        pros::delay(50);
    }
}