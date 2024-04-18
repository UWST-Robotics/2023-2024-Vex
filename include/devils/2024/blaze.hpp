#pragma once

#include "../devils.h"
#include "systems/intakeSystem.hpp"
#include "systems/launcherSystem.hpp"

namespace devils
{
    /**
     * Represents the Blaze robot and all of its subsystems.
     */
    struct Blaze : public Robot
    {
        /**
         * Creates a new instance of Blaze.
         */
        Blaze()
        {
            // Init Differential Wheel Odometry
            wheelOdom.setTicksPerRevolution(TICKS_PER_REVOLUTION);
            
            // Add Stats
            StatsRenderer* statsRenderer = mainDisplay.getRenderer<StatsRenderer>();
            statsRenderer->useOdomSource(&wheelOdom);
            statsRenderer->useChassis(&chassis);

            // Run Display
            mainDisplay.runAsync();
        }

        void autonomous() override
        {
            // Set Debug Speed
            chassis.setSpeed(0.8);

            // Reset Auto Controller
            autoController.reset();

            while (true)
            {
                // Update Odometry
                wheelOdom.update(chassis);

                // Run Auto Controller
                autoController.update();

                pros::delay(20);
            }
        }

        void opcontrol() override
        {
            // Reset Speed
            chassis.setSpeed(1.0);

            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                double rightY = mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;
                bool topBumper = mainController.get_digital(DIGITAL_L1) || mainController.get_digital(DIGITAL_R1);
                bool bottomBumper = mainController.get_digital(DIGITAL_L2) || mainController.get_digital(DIGITAL_R2);

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 2.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 2.0, 0.1);
                rightY = JoystickCurve::curve(rightY, 2.0, 0.1);

                // Drive the Robot
                chassis.move(leftY, leftX);
                launcher.fireVoltage(rightY, -rightY);

                // Intake
                if (topBumper)
                    intake.intake();
                else if (bottomBumper)
                    intake.outtake();
                else
                    intake.stop();

                // Update Odometry
                wheelOdom.update(leftSensor, rightSensor);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void disabled() override
        {
            // Climb
            // blocker.retract(true);
        }

    private:
        // V5 Ports
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {-11, 12, 3, -2, -1};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {-19, 20, -8, 9, 10};
        static constexpr std::initializer_list<int8_t> INTAKE_MOTOR_PORTS = {15, -6};
        static constexpr uint8_t LEFT_FLYWHEEL_PORT = 4;
        static constexpr uint8_t RIGHT_FLYWHEEL_PORT = 7;
        static constexpr uint8_t LEFT_ROTATION_SENSOR_PORT = 18;
        static constexpr uint8_t RIGHT_ROTATION_SENSOR_PORT = 13;

        // Geometry
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 15.0;                            // in
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (60.0 / 36.0); // ticks

        // Subsystems
        TankChassis chassis = TankChassis(L_MOTOR_PORTS, R_MOTOR_PORTS);
        LauncherSystem launcher = LauncherSystem(LEFT_FLYWHEEL_PORT, RIGHT_FLYWHEEL_PORT);
        IntakeSystem intake = IntakeSystem(INTAKE_MOTOR_PORTS);

        // Sensors
        RotationSensor leftSensor = RotationSensor("Blaze.LeftRotation", LEFT_ROTATION_SENSOR_PORT);
        RotationSensor rightSensor = RotationSensor("Blaze.RightRotation", RIGHT_ROTATION_SENSOR_PORT);

        // Odometry
        DifferentialWheelOdometry wheelOdom = DifferentialWheelOdometry(WHEEL_RADIUS, WHEEL_BASE);

        // Controller
        PursuitController pursuitController = PursuitController(chassis, wheelOdom);
        ControllerList autoController = ControllerList({&pursuitController}, true);

        // Display
        Display mainDisplay = Display({new GridRenderer(),
                                       new FieldRenderer(),
                                       new OdomRenderer(&wheelOdom),
                                       new ControlRenderer(&autoController, &wheelOdom),
                                       new StatsRenderer()});
    };
}