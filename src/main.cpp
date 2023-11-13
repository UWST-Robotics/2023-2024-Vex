#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	Logger::init();
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
void autonomous()
{
	Logger::warn("Starting autocontrol");
	TestBot robot = TestBot();

	// Auto Controller
	PursuitController controller(robot.chassis, robot.motionProfile, robot.odometry);
	// OpenLoopController controller(robot.chassis, robot.motionProfile);
	controller.restart();

	// Display
	OdomRenderer odomRenderer(&robot.odometry);
	MotionRenderer motionRenderer(&robot.motionProfile);
	ControlRenderer controlRenderer(&controller);
	Display autoDisplay = Display({&odomRenderer, &motionRenderer, &controlRenderer});

	// Loop
	while (true)
	{
		robot.updateOdometry();

		controller.update();
		autoDisplay.update();

		// Delay to prevent the CPU from being overloaded
		pros::delay(20);
	}
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
void opcontrol()
{
	Logger::warn("Starting opcontrol");
	TestBot robot = TestBot();

	// Teleop Controller
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	// Display
	OdomRenderer odomRenderer(&robot.odometry);
	MotionRenderer motionRenderer(&robot.motionProfile);
	Display teleopDisplay = Display({&odomRenderer, &motionRenderer});

	// Loop
	while (true)
	{
		// Get Controller Values
		double leftY = master.get_analog(ANALOG_LEFT_Y) / 127.0;
		double rightX = master.get_analog(ANALOG_RIGHT_X) / 127.0;
		bool fire = master.get_digital(DIGITAL_R1);
		bool extend = master.get_digital(DIGITAL_R2);
		bool intake = master.get_digital(DIGITAL_L1);
		bool outtake = master.get_digital(DIGITAL_L2);
		bool climb = master.get_digital(DIGITAL_UP);

		/*
		// Catapult
		if (fire)
			robot.catapult.fire();
		else
			robot.catapult.stop();

		// Wings
		if (extend)
			robot.wings.extend();
		else
			robot.wings.retract();

		// Intake
		if (intake)
			robot.intake.intake();
		else if (outtake)
			robot.intake.outtake();
		else
			robot.intake.stop();
		*/

		// LEDs
		robot.leds.scale(1 - abs(leftY));

		// Arcade Drive
		robot.chassis.move(leftY * 0.3, rightX * 0.3);

		// Odometry
		robot.updateOdometry();

		// Simulation
		teleopDisplay.update();

		// Delay to prevent the CPU from being overloaded
		pros::delay(20);
	}
}
