#pragma once
#include "pros/motors.hpp"
#include <errno.h>
#include "devils/utils/logger.hpp"

namespace devils
{
    class LEDSystem
    {
    public:
        /**
         * Controls the LED lights on the robot.
         */
        LEDSystem() : leds{
                          pros::ADIDigitalOut(1),
                          pros::ADIDigitalOut(2),
                          pros::ADIDigitalOut(3),
                          pros::ADIDigitalOut(4),
                          pros::ADIDigitalOut(5),
                          pros::ADIDigitalOut(6),
                          pros::ADIDigitalOut(7),
                          pros::ADIDigitalOut(8)}
        {
            if (errno != 0)
                Logger::error("LEDSystem: led ports are invalid");
        }

        /**
         * Turns on all of the LEDs.
         */
        void turnOn()
        {
            for (auto led : leds)
                led.set_value(true);
        }

        /**
         * Turns off all of the LEDs.
         */
        void turnOff()
        {
            for (auto led : leds)
                led.set_value(false);
        }

        /**
         * Scales the LEDs from 0 to 1.
         * @param value The value to scale the LEDs to.
         */
        void scale(double value, bool invert = false)
        {
            int count = (int)round(8 * value);
            for (int i = 0; i < count; i++)
                leds[i].set_value(!invert);
            for (int i = count; i < 8; i++)
                leds[i].set_value(invert);
        }

    private:
        pros::ADIDigitalOut leds[8];
    };
}