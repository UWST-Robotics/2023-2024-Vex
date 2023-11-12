#pragma once

namespace devils
{
    class Units
    {
    public:
        static double inToMeters(double inches)
        {
            return inches * 0.0254;
        }

        static double metersToIn(double meters)
        {
            return meters / 0.0254;
        }

        static double diffRad(double radiansA, double radiansB)
        {
            double diff = radiansA - radiansB;
            diff += (diff > M_PI) ? -2 * M_PI : ((diff < -M_PI) ? 2 * M_PI : 0);
            return diff;
        }

        static double degToRad(double degrees)
        {
            return degrees * 0.0174533;
        }

        static double radToDeg(double radians)
        {
            return radians / 0.0174533;
        }
    };
}