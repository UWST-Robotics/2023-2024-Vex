#pragma once

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace devils
{
    /**
     * A class containing a variety of unit conversion utilities.
     */
    class Units
    {
    public:
        /**
         * Converts inches to meters.
         * @param inches The inches to convert.
         */
        static double inToMeters(double inches)
        {
            return inches * 0.0254;
        }

        /**
         * Converts meters to inches.
         * @param meters The meters to convert.
         */
        static double metersToIn(double meters)
        {
            return meters / 0.0254;
        }

        /**
         * Compares the difference of two radian angles.
         * @param radiansA The first angle in radians.
         * @param radiansB The second angle in radians.
         * @return The difference between the two angles in radians.
         */
        static double diffRad(double radiansA, double radiansB)
        {
            double diff = radiansA - radiansB;
            diff += (diff > M_PI) ? -2 * M_PI : ((diff < -M_PI) ? 2 * M_PI : 0);
            return diff;
        }

        /**
         * Converts degrees to radians.
         * @param degrees The degrees to convert.
         */
        static double degToRad(double degrees)
        {
            return degrees * (M_PI / 180.0);
        }

        /**
         * Converts radians to degrees.
         * @param radians The radians to convert.
         */
        static double radToDeg(double radians)
        {
            return radians * (180.0 / M_PI);
        }

        /**
         * Normalizes an angle in radians to be between 0 and 2 * PI.
         * @param radians The angle in radians to normalize.
         */
        static double normalizeRadians(double radians)
        {
            return std::fmod(radians + 2 * M_PI, 2 * M_PI);
        }
    };
}