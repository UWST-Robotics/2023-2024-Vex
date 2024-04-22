#pragma once

#include "../devils.h"
#include "systems/intakeSystem.hpp"
#include "systems/launcherSystem.hpp"
#include "control/blazeAutoController.hpp"

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
            // Fix Bounce Controller
            bounceController.setSpeeds(0.4, -0.4);
            bounceController.setDurations(100, 100);

            // Add Renderers
            statsRenderer->useOdomSource(&wheelOdom);
            statsRenderer->useChassis(&chassis);
            autoController.usePathRenderer(*pathRenderer);

            // Run Odom
            wheelOdom.useIMU(imu);
            wheelOdom.runAsync();

            // Run Display
            mainDisplay.runAsync();
        }

        void autonomous() override
        {
            // Init Odom
            wheelOdom.setPose(*autoController.getStartingPose());

            // Set Speed
            chassis.setSpeed(CHASSIS_AUTO_FORWARD, CHASSIS_AUTO_TURN);

            // Calibrate IMU
            imu.calibrate();
            imu.waitUntilCalibrated();

            // Reset Auto Controller
            autoController.reset();

            EventTimer pauseTimer = EventTimer();
            EventTimer bounceTimer = EventTimer();

            while (true)
            {
                // Run Auto Controller
                if (pauseTimer.getRunning())
                    chassis.stop();
                else if (bounceTimer.getRunning())
                    bounceController.update();
                else
                    autoController.update();

                // Handle Auto Controller Events
                PathEvents *events = autoController.getState().events;
                for (PathEvent event : *events)
                {
                    // Post-Pause Events
                    bool isPaused = pauseTimer.getRunning() || bounceTimer.getRunning();
                    if (event.params == "afterPause" && isPaused)
                        continue;

                    // Intake
                    else if (event.name == "intake")
                    {
                        outerIntake.intake();
                        innerIntake.intake();
                    }
                    else if (event.name == "outtake")
                    {
                        outerIntake.outtake();
                        innerIntake.outtake();
                    }
                    else if (event.name == "stopIntake")
                    {
                        outerIntake.stop();
                        innerIntake.stop();
                    }

                    // Raise/Lower Intake
                    else if (event.name == "raiseIntake")
                    {
                        outerIntake.retract();
                    }
                    else if (event.name == "lowerIntake")
                    {
                        outerIntake.extend();
                    }

                    // Launcher
                    else if (event.name == "fire")
                    {
                        launcher.firePID();
                    }
                    else if (event.name == "stopLauncher")
                    {
                        launcher.stop();
                    }

                    // Other
                    else if (event.name == "pause")
                    {
                        pauseTimer.start(event.id, std::stoi(event.params));
                    }
                    else if (event.name == "setSpeed")
                    {
                        chassis.setSpeed(std::stod(event.params) * CHASSIS_AUTO_FORWARD, CHASSIS_AUTO_TURN);
                    }
                    else if (event.name == "bounce")
                    {
                        bounceTimer.start(event.id, std::stoi(event.params));
                    }
                    else if (event.name == "alignToAngle")
                    {
                        double angleDeg = std::stod(event.params);
                        double angleRad = Units::degToRad(angleDeg);
                        double deltaAngle = Units::diffRad(angleRad, imu.getHeading());

                        bounceController.setRotation(deltaAngle);
                        if (pauseTimer.getRunning())
                            chassis.move(0.0, deltaAngle);
                    }
                    else
                    {
                        Logger::warn("Unknown Event: " + event.name);
                    }
                }

                // Pause
                pros::delay(20);
            }
        }

        void opcontrol() override
        {
            // Reset Speed
            chassis.setSpeed(1.0, 1.0);

            // Control
            bool intakeExtended = false;
            bool runFlywheel = false;

            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                bool intakeButton = mainController.get_digital(DIGITAL_L1) || mainController.get_digital(DIGITAL_R1);
                bool outtakeButton = mainController.get_digital(DIGITAL_L2) || mainController.get_digital(DIGITAL_R2);
                bool extendButton = mainController.get_digital_new_press(DIGITAL_A);
                bool launchButton = mainController.get_digital_new_press(DIGITAL_B);
                bool upButon = mainController.get_digital_new_press(DIGITAL_UP);
                bool downButton = mainController.get_digital_new_press(DIGITAL_DOWN);

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 2.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 2.0, 0.1);

                // Drive the Robot
                chassis.move(leftY, leftX);

                // Flywheel
                if (launchButton)
                    runFlywheel = !runFlywheel;
                if (runFlywheel)
                    launcher.firePID();
                else
                    launcher.stop();

                // Adjust Flywheel
                if (upButon)
                    launcher.setSetpoint(launcher.getSetpoint() + 5);
                if (downButton)
                    launcher.setSetpoint(launcher.getSetpoint() - 5);
                mainController.set_text(0, 0, std::to_string((int)launcher.getSetpoint()) + "rpm - " + std::to_string((int)launcher.getCurrentVelocity()) + "rpm");

                // Outer Intake
                if (intakeButton)
                    outerIntake.intake();
                else if (outtakeButton)
                    outerIntake.outtake();
                else
                    outerIntake.stop();

                // Inner Intake
                if (intakeButton)
                    innerIntake.intake();
                else if (outtakeButton)
                    innerIntake.outtake();
                else
                    innerIntake.stop();

                // Intake Extend
                if (extendButton)
                    intakeExtended = !intakeExtended;
                if (intakeExtended)
                    outerIntake.extend();
                else
                    outerIntake.retract();

                // Delay to prevent the CPU from being overloaded
                pros::delay(10);
            }
        }

        void disabled() override
        {
            // Climb
            // blocker.retract(true);
        }

    private:
        // V5 Ports
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {19, -20, 8, -9, -10};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {11, -12, -3, 2, 1};
        static constexpr std::initializer_list<int8_t> INNER_INTAKE_MOTOR_PORTS = {-6};
        static constexpr std::initializer_list<int8_t> OUTER_INTAKE_MOTOR_PORTS = {15};
        static constexpr uint8_t LEFT_FLYWHEEL_PORT = 4;
        static constexpr uint8_t RIGHT_FLYWHEEL_PORT = 7;
        static constexpr int8_t LEFT_ROTATION_SENSOR_PORT = -18;
        static constexpr int8_t RIGHT_ROTATION_SENSOR_PORT = 13;
        static constexpr uint8_t IMU_SENSOR_PORT = 14;

        // Speeds
        static constexpr double CHASSIS_AUTO_FORWARD = 0.3; // % speed
        static constexpr double CHASSIS_AUTO_TURN = 0.7;    // % speed

        // ADI Ports
        static constexpr std::initializer_list<uint8_t> OUTER_INTAKE_PNEUMATIC_PORTS = {7, 8};

        // Geometry
        static constexpr double WHEEL_RADIUS = 1.0; // in
        static constexpr double WHEEL_BASE = 4.0;   // in

        // Subsystems
        TankChassis chassis = TankChassis(L_MOTOR_PORTS, R_MOTOR_PORTS);
        LauncherSystem launcher = LauncherSystem(LEFT_FLYWHEEL_PORT, RIGHT_FLYWHEEL_PORT);
        IntakeSystem innerIntake = IntakeSystem("InnerIntake", INNER_INTAKE_MOTOR_PORTS);
        IntakeSystem outerIntake = IntakeSystem("OuterIntake", OUTER_INTAKE_MOTOR_PORTS, OUTER_INTAKE_PNEUMATIC_PORTS);

        // Sensors
        IMU imu = IMU("Blaze.IMU", IMU_SENSOR_PORT);
        RotationSensor leftSensor = RotationSensor("Blaze.LeftRotation", LEFT_ROTATION_SENSOR_PORT);
        RotationSensor rightSensor = RotationSensor("Blaze.RightRotation", RIGHT_ROTATION_SENSOR_PORT);

        // Odometry
        DifferentialWheelOdometry wheelOdom = DifferentialWheelOdometry(leftSensor, rightSensor, WHEEL_RADIUS, WHEEL_BASE);

        // Controller
        BlazeAutoController autoController = BlazeAutoController(chassis, wheelOdom);
        BounceController bounceController = BounceController(chassis);

        // Display
        Display mainDisplay = Display({new GridRenderer(),
                                       new FieldRenderer(),
                                       new OdomRenderer(&wheelOdom),
                                       new ControlRenderer(&autoController, &wheelOdom),
                                       new PathRenderer(nullptr),
                                       new StatsRenderer()});
        StatsRenderer *statsRenderer = mainDisplay.getRenderer<StatsRenderer>();
        PathRenderer *pathRenderer = mainDisplay.getRenderer<PathRenderer>();
    };
}