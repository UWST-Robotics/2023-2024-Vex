#pragma once

#include "../devils.h"
#include "systems/intakeSystem.hpp"

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
            // gps.setPose();

            // Run Display
            mainDisplay.runAsync();
        }

        void autonomous() override
        {
        }

        void opcontrol() override
        {
            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                // bool leftWing = mainController.get_digital(DIGITAL_L1) || mainController.get_digital(DIGITAL_L2);
                // bool rightWing = mainController.get_digital(DIGITAL_R1) || mainController.get_digital(DIGITAL_R2);
                // bool blockerUp = mainController.get_digital_new_press(DIGITAL_X);
                // bool blockerDown = mainController.get_digital(DIGITAL_B);
                double intakeValue = mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;

                // Curve Joystick Inputs
                // leftY = JoystickCurve::square(JoystickCurve::dlerp(0.1, 0.3, 1.0, leftY));
                // leftX = JoystickCurve::square(leftX);
                leftY = JoystickCurve::curve(leftY, 2.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 2.0, 0.05);

                // Drive the Robot
                intake.intake(intakeValue);
                chassis.move(leftY, leftX);

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
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {10, -19, -8, 18};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {-11, 12, -1, 2};
        static constexpr uint8_t INTAKE_MOTOR_PORT = 9;
        static constexpr uint8_t IMU_PORT = 5;
        static constexpr uint8_t GPS_PORT = 5; // <<< ??
        static constexpr uint8_t STORAGE_SENSOR_PORT = 20;

        // Geometry
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 12.0;                            // in
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (60.0 / 36.0); // ticks

        static constexpr double GPS_OFFSET_X = 0.0;         // in
        static constexpr double GPS_OFFSET_Y = -7.0;        // in
        static constexpr double GPS_OFFSET_ROTATION = M_PI; // rad

        // Subsystems
        TankChassis chassis = TankChassis(L_MOTOR_PORTS, R_MOTOR_PORTS);
        IntakeSystem intake = IntakeSystem(INTAKE_MOTOR_PORT);

        // Sensors
        GPS gps = GPS("PepperJack.GPS", GPS_PORT);
        IMU imu = IMU("PepperJack.IMU", IMU_PORT);
        OpticalSensor storageSensor = OpticalSensor("PepperJack.StorageSensor", STORAGE_SENSOR_PORT);

        // Odometry
        DifferentialWheelOdometry wheelOdom = DifferentialWheelOdometry(WHEEL_RADIUS, WHEEL_BASE);
        ComplementaryFilterOdom fusedOdom = ComplementaryFilterOdom(&gps, &wheelOdom, 0.01);

        // Display
        Display mainDisplay = Display({new GridRenderer(),
                                       new FieldRenderer(),
                                       new OdomRenderer(&fusedOdom),
                                       new StatsRenderer()});
    };
}