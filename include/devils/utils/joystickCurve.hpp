#pragma once
#include "okapi/api/util/logging.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include <string>
#include <unistd.h>

namespace devils
{
    /**
     * Represents a joystick curve.
     */
    struct JoystickCurve
    {
        /**
         * Linearly interpolates a value with a deadzone
         * @param deadzone The deadzone of the joystick.
         * @param min The minimum value of the joystick.
         * @param max The maximum value of the joystick.
         * @param val The value of the joystick.
         * @return The interpolated value.
         */
        static double dlerp(double deadzone, double min, double max, double val)
        {
            if (val > -deadzone && val < deadzone)
                return 0;
            return lerp(min, max, val);
        }

        /**
         * Linearly interpolates a value.
         * @param a The minimum value.
         * @param b The maximum value.
         * @param val The value to interpolate.
         * @return The interpolated value.
         */
        static double lerp(double a, double b, double val)
        {
            if (val < 0)
                return ((1 + val) * a - val * b) * -1;
            return (1 - val) * a + val * b;
        }

        /**
         * Curves on an arbitrary power curve.
         * @param val The value to curve.
         * @param power The power to curve by.
         * @return The curved value.
         */
        static double pow(double val, double power)
        {
            return std::pow(std::abs(val), power) * (val < 0 ? -1 : 1);
        }

        /**
         * Curves on an arbitrary curve with a deadzone.
         * @param val The value to curve.
         * @param power The power to curve by.
         * @param deadzone The range of which the joystick is considered 0.
         * @return The curved value.
         */
        static double curve(double val, double power, double deadzone)
        {
            if (val > -deadzone && val < deadzone)
                return 0;
            return pow(val, power);
        }

        /**
         * Curves on a square curve.
         * @param val The value to curve.
         * @return The curved value.
         */
        static double square(double val)
        {
            return val * val * (val < 0 ? -1 : 1);
        }

        /**
         * Curves on a cubic curve.
         * @param val The value to curve.
         * @return The curved value.
         */
        static double cubic(double val)
        {
            return val * val * val;
        }

    private:
        JoystickCurve() = delete;
    };
}