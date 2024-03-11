#pragma once

#include "../devils.h"

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
            : chassis(L_MOTOR_PORTS, R_MOTOR_PORTS),
              imu("Blaze.IMU", IMU_PORT),
              intake(INTAKE_PORT, INTAKE_ACTUATOR_PORT),
              wings(LEFT_WINGS_PORT, RIGHT_WINGS_PORT),
              odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION)
        {
            odometry.useIMU(&imu);

            // Motion Profile
            Logger::info("Generating Motion Profile...");
            auto generator = LinearGenerator();
            generator.generate(&motionProfile);
        }

        void disabled() override
        {
            intake.raise();
        }

        void autonomous() override
        {
            // Game Controller
            pros::Controller master(pros::E_CONTROLLER_MASTER);

            // Controller/Odom
            LinearController pursuitController = LinearController(chassis, motionProfile, odometry);
            AutoTimer pauseTimer;

            pursuitController.reset();

            // Display
            OdomRenderer odomRenderer(&odometry);
            MotionRenderer motionRenderer(&motionProfile);
            ControlRenderer controlRenderer(&pursuitController);
            Display teleopDisplay = Display({&odomRenderer, &motionRenderer, &controlRenderer});

            // Enable Ramping
            chassis.getLeftMotors()->setRampRate(4);
            chassis.getRightMotors()->setRampRate(4);

            // Run
            while (true)
            {
                // Debug
                master.set_text(0, 0, std::to_string(pursuitController.getCurrentIndex()));

                // Handle Events
                auto currentEvents = pursuitController.getCurrentEvents();
                for (int i = 0; i < currentEvents.size(); i++)
                {
                    auto currentEvent = currentEvents[i];
                    Logger::info("Event: " + currentEvent.name);

                    if (currentEvent.name == "intake")
                    {
                        intake.intake();
                        if (currentEvent.params != "")
                            pauseTimer.start(currentEvent.id, std::stoi(currentEvent.params));
                    }
                    else if (currentEvent.name == "outtake")
                    {
                        intake.outtake();
                        if (currentEvent.params != "")
                            pauseTimer.start(currentEvent.id, std::stoi(currentEvent.params));
                    }
                    else if (currentEvent.name == "stopIntake")
                    {
                        intake.stop();
                    }
                    else if (currentEvent.name == "closeWings")
                    {
                        wings.retractLeft();
                        wings.retractRight();
                    }
                    else if (currentEvent.name == "wings")
                    {
                        wings.extendLeft();
                        wings.extendRight();
                    }
                    else if (currentEvent.name == "rightWing")
                    {
                        wings.extendRight();
                        if (currentEvent.params == "closeLeft")
                            wings.retractLeft();
                    }
                    else if (currentEvent.name == "leftWing")
                    {
                        wings.extendLeft();
                        if (currentEvent.params == "closeRight")
                            wings.retractRight();
                    }
                    else if (currentEvent.name == "pause")
                    {
                        pauseTimer.start(currentEvent.id, std::stoi(currentEvent.params));
                    }
                    else if (currentEvent.name == "liftIntake")
                    {
                        intake.raise();
                    }
                    else if (currentEvent.name == "lowerIntake")
                    {
                        intake.lower();
                    }
                    else if (currentEvent.name == "wack")
                    {
                        wings.autoExtendRight();
                        pauseTimer.start(currentEvent.id, std::stoi(currentEvent.params));
                        if (!pauseTimer.getRunning())
                            wings.retractRight();
                    }
                    else if (currentEvent.name == "pause")
                    {
                        pauseTimer.start(currentEvent.id, std::stoi(currentEvent.params));
                    }
                    else if (currentEvent.name == "abortIfDriver")
                    {
                        bool isDriverControl = !pros::competition::is_autonomous() && !pros::competition::is_disabled();
                        if (isDriverControl)
                            return;
                    }
                }

                // Check For Fast/Slow
                bool isFast = false;
                bool isSlow = false;
                for (int i = 0; i < currentEvents.size(); i++)
                {
                    auto currentEvent = currentEvents[i];
                    if (currentEvent.name == "fast")
                        isFast = true;
                    else if (currentEvent.name == "slow")
                        isSlow = true;
                }

                // Set Max Speed
                if (isFast)
                    pursuitController.setSpeed(1.0);
                else if (isSlow)
                    pursuitController.setSpeed(0.25);
                else
                    pursuitController.setSpeed(0.5);

                // Update Odometry
                odometry.update(&chassis);

                // Anti-Tip
                if (imu.getPitch() > 25)
                    chassis.move(-1.0, 0.0);
                // Run Pursuit Controller
                // else if (!pauseTimer.getRunning())
                //    pursuitController.update();
                else
                    pursuitController.pause();

                // Run Display
                teleopDisplay.update();

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void opcontrol() override
        {
            // Teleop Controller
            pros::Controller master(pros::E_CONTROLLER_MASTER);

            // Autonomous Code
            bool abortAuto = false;

            // Controller/Odom
            LinearController pursuitController = LinearController(chassis, motionProfile, odometry);
            // pursuitController.restart();

            // Enable Ramping
            chassis.getLeftMotors()->setRampRate(4);
            chassis.getRightMotors()->setRampRate(4);

            // Run
            while (master.get_digital(DIGITAL_A) && !abortAuto)
            {
                // Handle Events
                auto currentEvents = pursuitController.getCurrentEvents();

                // Check For Fast/Slow
                bool isFast = false;
                bool isSlow = false;
                for (int i = 0; i < currentEvents.size(); i++)
                {
                    auto currentEvent = currentEvents[i];
                    if (currentEvent.name == "fast")
                        isFast = true;
                    else if (currentEvent.name == "slow")
                        isSlow = true;
                    else if (currentEvent.name == "abortIfDriver")
                    {
                        chassis.stop();
                        abortAuto = true;
                        master.rumble("-");
                        break;
                    }
                }

                if (abortAuto)
                    break;

                // Set Max Speed
                if (isFast)
                    pursuitController.setSpeed(1.0);
                else if (isSlow)
                    pursuitController.setSpeed(0.25);
                else
                    pursuitController.setSpeed(0.5);

                // Update Odometry
                odometry.update(&chassis);

                // Run Pursuit Controller
                // pursuitController.update();

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }

            // Wings
            bool leftWing = false;
            bool rightWing = false;

            // Loop
            while (true)
            {
                // Controller
                double leftY = master.get_analog(ANALOG_LEFT_Y) / 127.0;
                double rightY = master.get_analog(ANALOG_RIGHT_Y) / 127.0;
                bool leftWingButton = master.get_digital_new_press(DIGITAL_L1);
                bool rightWingButton = master.get_digital_new_press(DIGITAL_R1);
                bool intakeUp = master.get_digital(DIGITAL_X);
                bool intakeDown = master.get_digital(DIGITAL_B);
                bool intakeInput = master.get_digital(DIGITAL_R2);
                bool outtakeInput = master.get_digital(DIGITAL_L2);

                // Curve Inputs
                leftY = Curve::square(Curve::dlerp(0.1, 0.3, 1.0, leftY));
                rightY = Curve::square(Curve::dlerp(0.1, 0.3, 1.0, rightY));

                // Intake Actuation
                if (intakeUp)
                    intake.raise();
                if (intakeDown)
                    intake.lower();

                // Intake Control
                if (intakeInput)
                    intake.intake();
                else if (outtakeInput)
                    intake.outtake();
                else
                    intake.stop();

                // Wing Buttons
                if (leftWingButton)
                    leftWing = !leftWing;
                if (rightWingButton)
                    rightWing = !rightWing;

                // Wing Control
                if (leftWing)
                    wings.extendLeft();
                else
                    wings.retractLeft();
                if (rightWing)
                    wings.extendRight();
                else
                    wings.retractRight();

                // Tank Drive
                chassis.moveTank(leftY, rightY);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        // Subsystems
        TankChassis chassis;
        WingSystem wings;
        ActuateIntakeSystem intake;

        // Autonomous
        TankWheelOdometry odometry;
        MotionProfile motionProfile;

        // Extra Sensors
        IMU imu;

        bool isDriverControl = false;

    private:
        // V5 Motors
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {-10, 6, -7, 8, -9};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {20, 19, 17, -18, -16};
        static constexpr uint8_t INTAKE_PORT = 4;

        // V5 Sensors
        static constexpr uint8_t IMU_PORT = 21;

        // ADI Ports
        static constexpr uint8_t INTAKE_ACTUATOR_PORT = 4;
        static constexpr uint8_t LEFT_WINGS_PORT = 2;
        static constexpr uint8_t RIGHT_WINGS_PORT = 3;

        // Odometry
        static constexpr double WHEEL_RADIUS = 1.625;                         // Radius of the wheel in inches
        static constexpr double WHEEL_BASE = 15.5;                            // Width of the robot in inches
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (60.0 / 36.0); // Number of ticks per revolution of the wheel * gear ratio
    };
}