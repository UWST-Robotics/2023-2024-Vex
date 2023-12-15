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
	robot = std::make_shared<PepperJack>();
	robot->generateMotionProfile();
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

	// Auto Controller
	PursuitController controller(robot->chassis, robot->motionProfile, robot->odometry);
	controller.restart();

	// Timer
	double pauseTimer = 0; // ms

	// Display
	OdomRenderer odomRenderer(&robot->odometry);
	MotionRenderer motionRenderer(&robot->motionProfile);
	ControlRenderer controlRenderer(&controller);
	Display autoDisplay = Display({&odomRenderer, &motionRenderer, &controlRenderer});

	// Loop
	while (true)
	{
		// Update Odometry
		robot->updateOdometry();

		// Drive Chassis
		if (pauseTimer <= 0)
			controller.update();
		else
			pauseTimer -= pros::millis();

		// Perform Actions
		auto events = controller.getCurrentEvents();
		for (auto event : events)
		{
			if (event.name == "pause")
				pauseTimer = std::stod(event.params);
			if (event.name == "runIntake")
				robot->intake.intake();
			if (event.name == "stopIntake")
				robot->intake.stop();
			if (event.name == "runOuttake")
				robot->intake.outtake();
			if (event.name == "openWings")
				robot->wings.extend();
			if (event.name == "closeWings")
				robot->wings.retract();
		}

		// Update Display
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

	// Teleop Controller
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	// Display
	OdomRenderer odomRenderer(&robot->odometry);
	MotionRenderer motionRenderer(&robot->motionProfile);
	Display teleopDisplay = Display({&odomRenderer, &motionRenderer});

	// Loop
	while (true)
	{
		// Controller
		double leftY = master.get_analog(ANALOG_LEFT_Y) / 127.0;
		double leftX = master.get_analog(ANALOG_LEFT_X) / 127.0;
		bool wings = master.get_digital(DIGITAL_R2);
		bool intake = master.get_digital(DIGITAL_L1);
		bool outtake = master.get_digital(DIGITAL_L2);
		bool block = master.get_digital(DIGITAL_A);

		// Curve Inputs
		leftY = Curve::square(Curve::dlerp(0.1, 0.3, 1.0, leftY));
		leftX = Curve::square(leftX);

		// Wings
		if (wings)
			robot->wings.extend();
		else
			robot->wings.retract();

		// Intake
		if (intake)
			robot->intake.intake();
		else if (outtake)
			robot->intake.outtake();
		else
			robot->intake.stop();

		// Arcade Drive
		robot->chassis.move(leftY, leftX);

		// Odometry
		robot->updateOdometry();

		// Simulation
		teleopDisplay.update();

		// Delay to prevent the CPU from being overloaded
		pros::delay(20);
	}
}
