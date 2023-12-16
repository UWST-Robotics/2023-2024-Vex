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
	double lastTime = pros::millis();

	// State
	int state = 0;

	// Loop
	while (true)
	{
		double deltaTime = pros::millis() - lastTime;
		lastTime = pros::millis();
		pauseTimer -= deltaTime;

		switch (state)
		{
		case 0:
			robot->blocker.extend();
			pauseTimer = 1000;
			state++;
			break;
		case 1:
			if (pauseTimer > 0)
				break;
			robot->chassis.move(0.3, 0.0);
			pauseTimer = 3000;
			state++;
			break;
		case 2:
			if (pauseTimer > 0)
				break;
			robot->chassis.move(0.0, 0.0);
			robot->blocker.retract(true);
			state++;
			break;
		case 3:
			robot->blocker.retract(true);
		}

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

	bool wasBlockerUp = false;
	bool isBlockerUp = false;

	// Loop
	while (true)
	{
		// Controller
		double leftY = master.get_analog(ANALOG_LEFT_Y) / 127.0;
		double leftX = master.get_analog(ANALOG_LEFT_X) / 127.0;
		bool leftWing = master.get_digital(DIGITAL_L2);
		bool rightWing = master.get_digital(DIGITAL_R2);
		bool blockerUp = master.get_digital(DIGITAL_X);
		bool isBlockerDown = master.get_digital(DIGITAL_B);
		double intakeValue = master.get_analog(ANALOG_RIGHT_Y) / 127.0;

		// Curve Inputs
		leftY = Curve::square(Curve::dlerp(0.1, 0.3, 1.0, leftY));
		leftX = Curve::square(leftX);

		// Wings
		if (leftWing)
			robot->wings.extendLeft();
		else
			robot->wings.retractLeft();
		if (rightWing)
			robot->wings.extendRight();
		else
			robot->wings.retractRight();

		// Intake
		robot->intake.intake(intakeValue);

		// Blocker
		if (blockerUp && !wasBlockerUp)
			isBlockerUp = !isBlockerUp;
		wasBlockerUp = blockerUp;

		if (!isBlockerUp || isBlockerDown)
			robot->blocker.retract(isBlockerDown);
		else
			robot->blocker.extend();

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
