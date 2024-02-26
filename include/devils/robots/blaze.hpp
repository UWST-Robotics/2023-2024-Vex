#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents the Blaze robot and all of its subsystems.
     */
    struct Blaze : public Robot
    {
    public:
        /**
         * Creates a new instance of Blaze.
         */
        Blaze()
            : chassis(L_MOTOR_PORTS, R_MOTOR_PORTS),
              imu("Blaze.IMU", IMU_PORT),
              launcher(LEFT_LAUNCHER_PORT, RIGHT_LAUNCHER_PORT, ARM_LAUNCHER_PORT),
              intake(INTAKE_PORT),
              odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION),
              deployer("Deployer", DEPLOY_PORT)
        {
            odometry.useIMU(&imu);

            // Motion Profile
            Logger::info("Generating Motion Profile...");
            auto generator = SplineGenerator();
            generator.generate(&motionProfile);
        }

        void autonomous()
        {
            // Loop
            while (true)
            {
                // Auto Fire Launcher
                launcher.autoFire();

                // Drop Intake
                intake.outtake();

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void opcontrol()
        {
            // Teleop Controller
            pros::Controller master(pros::E_CONTROLLER_MASTER);

            // Loop
            while (true)
            {
                // Controller
                double leftY = master.get_analog(ANALOG_LEFT_Y) / 127.0;
                double rightY = master.get_analog(ANALOG_RIGHT_Y) / 127.0;
                bool intakeInput = master.get_digital(DIGITAL_R1);
                bool outtakeInput = master.get_digital(DIGITAL_R2);
                bool fireLauncherA = master.get_digital(DIGITAL_L1);
                bool fireLauncherB = master.get_digital(DIGITAL_L2);
                bool lowerArm = master.get_digital(DIGITAL_A);
                bool deployPneumatic = master.get_digital(DIGITAL_X);
                bool increaseSpeed = master.get_digital_new_press(DIGITAL_UP);
                bool decreaseSpeed = master.get_digital_new_press(DIGITAL_DOWN);
                bool increaseDelta = master.get_digital_new_press(DIGITAL_RIGHT);
                bool decreaseDelta = master.get_digital_new_press(DIGITAL_LEFT);

                // Curve Inputs
                leftY = Curve::square(Curve::dlerp(0.1, 0.3, 1.0, leftY));
                rightY = Curve::square(Curve::dlerp(0.1, 0.3, 1.0, rightY));

                // Launcher
                if (fireLauncherA || fireLauncherB)
                    launcher.fire();
                else
                    launcher.stop();

                // Arm
                if (lowerArm)
                    launcher.lowerArm();
                else
                    launcher.raiseArm();

                // Deploy Pneumatic
                if (deployPneumatic)
                    deployer.extend();
                else
                    deployer.retract();

                // Speed Change
                if (increaseSpeed)
                    launcher.increaseSpeed();
                if (decreaseSpeed)
                    launcher.decreaseSpeed();
                if (increaseDelta)
                    launcher.increaseDelta();
                if (decreaseDelta)
                    launcher.decreaseDelta();

                // Update Display
                if (increaseSpeed || decreaseSpeed || increaseDelta || decreaseDelta)
                {
                    std::string launchSpeed = std::to_string((int)(launcher.getSpeed() * 100)) + "% [" + std::to_string(launcher.getDelta()) + "]";
                    master.set_text(1, 1, launchSpeed);
                }

                // Intake
                if (intakeInput)
                    intake.intake();
                else if (outtakeInput)
                    intake.outtake();
                else
                    intake.stop();

                // Tank Drive
                chassis.moveTank(leftY, rightY);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        // Subsystems
        TankChassis chassis;
        IntakeSystem intake;
        LauncherSystem launcher;

        // Autonomous
        TankWheelOdometry odometry;
        MotionProfile motionProfile;

        // Extra Sensors
        IMU imu;
        ScuffPneumatic deployer;

    private:
        // V5 Motors
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {-7, 8, -9, 10};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {11, -12, -15, 16};
        static constexpr uint8_t LEFT_LAUNCHER_PORT = 13;
        static constexpr uint8_t RIGHT_LAUNCHER_PORT = 14;
        static constexpr uint8_t INTAKE_PORT = -20;

        // V5 Sensors
        static constexpr uint8_t IMU_PORT = 21;

        // ADI Ports
        static constexpr uint8_t ARM_LAUNCHER_PORT = 3;
        static constexpr uint8_t DEPLOY_PORT = 2;

        // Odometry
        static constexpr double WHEEL_RADIUS = 1.625;                         // Radius of the wheel in inches
        static constexpr double WHEEL_BASE = 15.5;                            // Width of the robot in inches
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (60.0 / 36.0); // Number of ticks per revolution of the wheel * gear ratio
    };
}