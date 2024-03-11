#pragma once
#include "../odom/pose.hpp"
#include "units.hpp"
#include <cmath>

namespace devils
{
    class PathUtils
    {
    public:
        /**
         * Linearly interpolates a value from a to b.
         * @param a The minimum value.
         * @param b The maximum value.
         * @param t The ratio between a and b. Values between 0 and 1.
         * @return The interpolated value.
         */
        static double cubicLerp(double a, double b, double t)
        {
            return a + (b - a) * t;
        }

        /**
         * Linearly interpolates a rotational value from a to b.
         * @param a The minimum value in radians.
         * @param b The maximum value in radians.
         * @param t The ratio between a and b. Values between 0 and 1.
         */
        static double rotationLerp(double a, double b, double t)
        {
            double aMod = std::fmod(a, 2 * M_PI);
            double bMod = std::fmod(b, 2 * M_PI);
            double diff = std::abs(aMod - bMod);
            if (diff > M_PI)
            {
                if (aMod > bMod)
                {
                    return Units::normalizeRadians(cubicLerp(aMod, bMod + 2 * M_PI, t));
                }
                else
                {
                    return Units::normalizeRadians(cubicLerp(aMod + 2 * M_PI, bMod, t));
                }
            }
            return Units::normalizeRadians(cubicLerp(aMod, bMod, t));
        }

        /**
         * Linearly interpolates a point from a to b.
         * @param a The minimum point.
         * @param b The maximum point.
         * @param t The ratio between a and b. Values between 0 and 1.
         * @return The interpolated point.
         */
        static Pose lerpPoints(Pose a, Pose b, double t)
        {
            return Pose{
                cubicLerp(a.x, b.x, t),
                cubicLerp(a.y, b.y, t),
                rotationLerp(a.rotation, b.rotation, t)};
        }

        /**
         * Quadratically interpolates a point from a to b to c.
         * @param a The minimum point.
         * @param b The middle point.
         * @param c The maximum point.
         * @param t The ratio between a and c. Values between 0 and 1.
         * @return The interpolated point.
         */
        static Pose quadraticLerpPoints(Pose a, Pose b, Pose c, double t)
        {
            Pose ab = lerpPoints(a, b, t);
            Pose bc = lerpPoints(b, c, t);
            return lerpPoints(ab, bc, t);
        }

        /**
         * Cubically interpolates a point from a to b to c to d.
         * @param a The minimum point.
         * @param b The middle point.
         * @param c The middle point.
         * @param d The maximum point.
         * @param t The ratio between a and d. Values between 0 and 1.
         * @return The interpolated point.
         */
        static Pose cubicLerpPoints(Pose a, Pose b, Pose c, Pose d, double t)
        {
            Pose abc = quadraticLerpPoints(a, b, c, t);
            Pose bcd = quadraticLerpPoints(b, c, d, t);
            return lerpPoints(abc, bcd, t);
        }
    };
}