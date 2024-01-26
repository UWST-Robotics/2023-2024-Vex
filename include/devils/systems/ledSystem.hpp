#pragma once
#include "../hardware/led.hpp"
#include "../utils/logger.hpp"

namespace devils
{
    class LEDSystem
    {
    public:
        /**
         * Controls the LED lights on the robot.
         */
        LEDSystem()
            : leds{LED("LEDSystem_1", 1),
                   LED("LEDSystem_2", 2),
                   LED("LEDSystem_3", 3),
                   LED("LEDSystem_4", 4),
                   LED("LEDSystem_5", 5),
                   LED("LEDSystem_6", 6),
                   LED("LEDSystem_7", 7),
                   LED("LEDSystem_8", 8)}
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
                led.enable();
        }

        /**
         * Turns off all of the LEDs.
         */
        void turnOff()
        {
            for (auto led : leds)
                led.disable();
        }

        /**
         * Scales the LEDs from 0 to 1.
         * @param value The value to scale the LEDs to.
         */
        void scale(double value, bool invert = false)
        {
            int count = (int)round(8 * value);
            for (int i = 0; i < count; i++)
                leds[i].setEnabled(!invert);
            for (int i = count; i < 8; i++)
                leds[i].setEnabled(invert);
        }

    private:
        LED leds[8];
    };
}