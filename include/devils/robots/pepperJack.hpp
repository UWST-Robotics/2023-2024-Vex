#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents the Pepper Jack robot and all of its subsystems.
     */
    struct PepperJack : public Robot
    {
    public:
        /**
         * Creates a new instance of PepperJack.
         */
        PepperJack()
            : chassis(L_MOTOR_PORTS, R_MOTOR_PORTS),
              imu("PepperJack.IMU", IMU_PORT),
              intake(INTAKE_MOTOR_PORT),
              storageSensor("PepperJack.StorageSensor", STORAGE_SENSOR_PORT),
              wings(LEFT_WINGS_PNEUMATIC_PORT, RIGHT_WINGS_PNEUMATIC_PORT),
              blocker(BLOCKER_PNEUMATIC_DOWN_PORT, BLOCKER_PNEUMATIC_UP_PORT),
              odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION)
        {
            odometry.useIMU(&imu);
            intake.useSensor(&storageSensor);

            // Motion Profile
            Logger::info("Generating Motion Profile...");
            auto generator = SplineGenerator();
            generator.generate(&motionProfile);
        }

        void autonomous()
        {
            // Controller/Odom
            PursuitController pursuitController = PursuitController(chassis, motionProfile, odometry);
            AutoTimer pauseTimer;

            pursuitController.restart();

            // Display
            OdomRenderer odomRenderer(&odometry);
            MotionRenderer motionRenderer(&motionProfile);
            ControlRenderer controlRenderer(&pursuitController);
            Display teleopDisplay = Display({&odomRenderer, &motionRenderer, &controlRenderer});

            // Run
            while (true)
            {
                // Handle Events
                auto currentEvents = pursuitController.getCurrentEvents();
                for (int i = 0; i < currentEvents.size(); i++)
                {
                    auto currentEvent = currentEvents[i];
                    if (currentEvent.name == "intake")
                    {
                        intake.intake();
                        if (currentEvent.params != "")
                            pauseTimer.start("intake", std::stoi(currentEvent.params));
                    }
                    else if (currentEvent.name == "outtake")
                    {
                        intake.outtake();
                        if (currentEvent.params != "")
                            pauseTimer.start("outtake", std::stoi(currentEvent.params));
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
                    else if (currentEvent.name == "liftClimber")
                    {
                        blocker.extend();
                    }
                    else if (currentEvent.name == "lowerClimber")
                    {
                        blocker.retract();
                    }
                    else if (currentEvent.name == "pause")
                    {
                        pauseTimer.start("pause", std::stoi(currentEvent.params));
                    }
                }

                if (currentEvents.size() <= 0)
                {
                    intake.stop();
                    wings.retractLeft();
                    wings.retractRight();
                    blocker.retract();
                }

                // Run Pursuit Controller
                odometry.update(&chassis);
                if (!pauseTimer.getRunning())
                    pursuitController.update();
                else
                    pursuitController.pause();

                // Run Display
                teleopDisplay.update();

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void opcontrol()
        {
            Logger::warn("Starting opcontrol");

            // Teleop Controller
            pros::Controller master(pros::E_CONTROLLER_MASTER);

            // Display
            PursuitController pursuitController = PursuitController(chassis, motionProfile, odometry);
            OdomRenderer odomRenderer(&odometry);
            MotionRenderer motionRenderer(&motionProfile);
            ControlRenderer controlRenderer(&pursuitController);
            Display teleopDisplay = Display({&odomRenderer, &motionRenderer, &controlRenderer});

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
                    wings.extendLeft();
                else
                    wings.retractLeft();
                if (rightWing)
                    wings.extendRight();
                else
                    wings.retractRight();

                // Intake
                intake.intake(intakeValue);

                // Blocker
                if (blockerUp && !wasBlockerUp)
                    isBlockerUp = !isBlockerUp;
                wasBlockerUp = blockerUp;

                if (!isBlockerUp || isBlockerDown)
                    blocker.retract(isBlockerDown);
                else
                    blocker.extend();

                // Arcade Drive
                chassis.move(leftY, leftX);

                // Odometry
                odometry.update(&chassis);

                // Simulation
                teleopDisplay.update();

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        // Subsystems
        TankChassis chassis;
        IntakeSystem intake;
        WingSystem wings;
        BlockerSystem blocker;

        // Autonomous
        TankWheelOdometry odometry;
        MotionProfile motionProfile;

        // Extra Sensors
        IMU imu;
        OpticalSensor storageSensor;

    private:
        // V5 Motors
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {1, 11, -2, -12};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {-10, -20, 9, 19};
        static constexpr uint8_t INTAKE_MOTOR_PORT = -21;

        // V5 Sensors
        static constexpr uint8_t IMU_PORT = 14;
        static constexpr uint8_t STORAGE_SENSOR_PORT = 8;

        // ADI Ports
        static constexpr uint8_t RIGHT_WINGS_PNEUMATIC_PORT = 4;
        static constexpr uint8_t LEFT_WINGS_PNEUMATIC_PORT = 3;
        static constexpr uint8_t BLOCKER_PNEUMATIC_DOWN_PORT = 2;
        static constexpr uint8_t BLOCKER_PNEUMATIC_UP_PORT = 1;

        // Odometry
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 12.0;                            // in
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (60.0 / 36.0); // ticks
    };
}