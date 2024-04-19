#pragma once

#include "../devils.h"
#include "systems/intakeSystem.hpp"
#include "systems/wingSystem.hpp"
#include "systems/pjAutoController.hpp"
#include "systems/testController.hpp"
#include "systems/blockerSystem.hpp"

namespace devils
{
    /**
     * Represents the Pepper Jack robot and all of its subsystems.
     */
    struct PepperJack : public Robot
    {
        /**
         * Creates a new instance of PepperJack.
         */
        PepperJack()
        {
            // Use IMU in Odometry
            wheelOdom.useIMU(imu);

            // Set GPS Offset
            gps.setOffset(GPS_OFFSET_X, GPS_OFFSET_Y, GPS_OFFSET_ROTATION);

            // Init Differential Wheel Odometry
            wheelOdom.setTicksPerRevolution(TICKS_PER_REVOLUTION);

            // Add Stats
            StatsRenderer *statsRenderer = mainDisplay.getRenderer<StatsRenderer>();
            statsRenderer->useOdomSource(&fusedOdom);
            statsRenderer->useChassis(&chassis);

            // Run Display
            mainDisplay.runAsync();
        }

        void autonomous() override
        {
            // Init Odom
            fusedOdom.setPose(*autoController.getStartingPose());

            // Set Speed
            chassis.setSpeed(CHASSIS_AUTO_FORWARD, CHASSIS_AUTO_TURN);

            // Calibrate IMU
            imu.calibrate();
            imu.waitUntilCalibrated();

            // Reset Auto Controller
            autoController.reset();

            EventTimer pauseTimer = EventTimer();

            while (true)
            {

                // Update Odometry
                gps.update();
                wheelOdom.update(chassis);
                fusedOdom.update();

                // Run Auto Controller
                if (pauseTimer.getRunning())
                    chassis.stop();
                else
                    autoController.update();

                // Handle Auto Controller Events
                PathEvents *events = autoController.getState().events;
                for (PathEvent event : *events)
                {

                    // Wings
                    if (event.name == "rightWing")
                    {
                        wings.extendRight();
                    }
                    else if (event.name == "leftWing")
                    {
                        wings.extendLeft();
                    }
                    else if (event.name == "closeWings")
                    {
                        wings.retractRight();
                        wings.retractLeft();
                    }
                    // Intake
                    else if (event.name == "intake")
                    {
                        intake.intake();
                    }
                    else if (event.name == "outtake")
                    {
                        intake.outtake();
                    }
                    else if (event.name == "stopIntake")
                    {
                        intake.stop();
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

            bool isBlockerUp = false;

            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                bool leftWing = mainController.get_digital(DIGITAL_L1) || mainController.get_digital(DIGITAL_L2);
                bool rightWing = mainController.get_digital(DIGITAL_R1) || mainController.get_digital(DIGITAL_R2);
                bool blockerUp = mainController.get_digital_new_press(DIGITAL_X);
                bool blockerDown = mainController.get_digital(DIGITAL_B);
                double intakeValue = -mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;

                // Curve Joystick Inputs
                // leftY = JoystickCurve::square(JoystickCurve::dlerp(0.1, 0.3, 1.0, leftY));
                // leftX = JoystickCurve::square(leftX);
                leftY = JoystickCurve::curve(leftY, 2.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 2.0, 0.05);

                // Drive the Robot
                intake.intake(intakeValue);
                chassis.move(leftY, leftX);

                // Wings
                if (leftWing)
                    wings.extendLeft();
                else
                    wings.retractLeft();
                if (rightWing)
                    wings.extendRight();
                else
                    wings.retractRight();

                // Blocker
                if (blockerUp)
                    isBlockerUp = !isBlockerUp;
                if (isBlockerUp)
                    blocker.extend();
                else
                    blocker.retract(blockerDown);

                // Update Odometry
                gps.update();
                wheelOdom.update(chassis);
                fusedOdom.update();

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
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {-12, 11, -9, 10};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {19, -20, 2, -1};
        static constexpr std::initializer_list<int8_t> INTAKE_MOTOR_PORTS = {-16};
        static constexpr uint8_t IMU_PORT = 4;
        static constexpr uint8_t GPS_PORT = 6;
        static constexpr uint8_t STORAGE_SENSOR_PORT = 7;

        // ADI Ports
        static constexpr uint8_t LEFT_WING_PORT = 1;
        static constexpr uint8_t RIGHT_WING_PORT = 2;
        static constexpr uint8_t BLOCKER_DOWN_PORT = 6;
        static constexpr uint8_t BLOCKER_UP_PORT = 3;

        // Geometry
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 12.0;                            // in
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (48.0 / 36.0); // ticks
        static constexpr double CHASSIS_AUTO_FORWARD = 0.3;                   // % speed
        static constexpr double CHASSIS_AUTO_TURN = 0.7;                      // % speed

        // GPS Offset
        static constexpr double GPS_OFFSET_X = 0.0;         // in
        static constexpr double GPS_OFFSET_Y = 6.0;         // in
        static constexpr double GPS_OFFSET_ROTATION = M_PI; // rad

        // Subsystems
        TankChassis chassis = TankChassis(L_MOTOR_PORTS, R_MOTOR_PORTS);
        IntakeSystem intake = IntakeSystem(INTAKE_MOTOR_PORTS);
        WingSystem wings = WingSystem(LEFT_WING_PORT, RIGHT_WING_PORT);
        BlockerSystem blocker = BlockerSystem(BLOCKER_DOWN_PORT, BLOCKER_UP_PORT);

        // Sensors
        GPS gps = GPS("PepperJack.GPS", GPS_PORT);
        IMU imu = IMU("PepperJack.IMU", IMU_PORT);
        OpticalSensor storageSensor = OpticalSensor("PepperJack.StorageSensor", STORAGE_SENSOR_PORT);

        // Odometry
        TransformOdom gpsOdom = TransformOdom(gps, false, false); // Transform GPS to match alliance side
        DifferentialWheelOdometry wheelOdom = DifferentialWheelOdometry(WHEEL_RADIUS, WHEEL_BASE);
        ComplementaryFilterOdom fusedOdom = ComplementaryFilterOdom(&gpsOdom, &wheelOdom, 0.02);

        // GameObjects
        GameObjectManager gameObjectManager = GameObjectManager();

        // Controller
        // PJAutoController autoController = PJAutoController(chassis, fusedOdom, gameObjectManager);
        TestController autoController = TestController(chassis, fusedOdom);

        // Display
        Display mainDisplay = Display({new GridRenderer(),
                                       new FieldRenderer(),
                                       new OdomRenderer(&fusedOdom),
                                       new ControlRenderer(&autoController, &fusedOdom),
                                       new PathRenderer(nullptr),
                                       new StatsRenderer()});
    };
}