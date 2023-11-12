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