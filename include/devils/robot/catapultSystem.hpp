#pragma once
#include "../hardware/smartMotor.hpp"
#include "../hardware/opticalSensor.hpp"

namespace devils
{
    /**
     * Controls the catapult system to launch triballs.
     */
    class CatapultSystem
    {
    public:
        /**
         * Creates a new catapult system.
         * @param motorPort The port of the catapult motor
         * @param winchPort The port of the winch motor
         */
        CatapultSystem(const int8_t motorPort, const int8_t winchPort)
            : catapultMotor("CatapultMotor", motorPort),
              winchMotor("WinchMotor", winchPort)
        {
        }

        /**
         * Runs the catapult motor (depends on the Optical Sensor if enabled)
         */
        void fire()
        {
            // Handle sensor
            if (enableSensor)
            {
                if (sensor->getProximity() > SENSOR_THRESHOLD)
                {
                    ballTimer -= pros::millis();
                    if (ballTimer <= 0)
                        forceFire();
                    else
                        stopLauncher();
                }
                else
                {
                    ballTimer = TIME_TO_LAUNCH;
                    stopLauncher();
                }
            }
            else
            {
                forceFire();
            }
        }

        /**
         * Runs the catapult motor regardless of the Optical Sensor
         */
        void forceFire()
        {
            catapultMotor.moveVoltage(FIRE_SPEED);
            isFiring = true;
        }

        /**
         * Stops the catapult motor
         */
        void stopLauncher()
        {
            catapultMotor.moveVoltage(0);
            isFiring = false;
        }

        /**
         * Stops the winch motor
         */
        void stopWinch()
        {
            winchMotor.moveVoltage(0);
        }

        /**
         * Stops both the catapult and winch motors
         */
        void stop()
        {
            stopLauncher();
            stopWinch();
        }

        /**
         * Extends the winch
         */
        void extend()
        {
            winchMotor.moveVoltage(WINCH_SPEED);
        }

        /**
         * Retracts the winch
         */
        void retract()
        {
            winchMotor.moveVoltage(-WINCH_SPEED);
        }

        /**
         * Returns true if the catapult is firing.
         */
        const bool getFiring()
        {
            return isFiring;
        }

        /**
         * Enables the Optical Sensor for the catapult.
         * Toggles the catapult if the Optical Sensor detects a triball.
         * @param sensor The Optical Sensor to use.
         */
        void useSensor(OpticalSensor *sensor)
        {
            enableSensor = true;
            this->sensor = sensor;
        }

    private:
        static constexpr double FIRE_SPEED = -1.0;      // -1 - 1 voltage
        static constexpr double WINCH_SPEED = 0.5;      // -1 - 1 voltage
        static constexpr double SENSOR_THRESHOLD = 0.5; // 0 - 1 proximity
        static constexpr double TIME_TO_LAUNCH = 500;   // ms

        double ballTimer = -1;

        bool isFiring = false;
        bool enableSensor = false;
        SmartMotor catapultMotor;
        SmartMotor winchMotor;
        OpticalSensor *sensor;
    };
}