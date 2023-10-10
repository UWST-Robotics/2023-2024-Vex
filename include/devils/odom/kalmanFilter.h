#pragma once

namespace devils
{
    class KalmanFilter
    {
    public:
        KalmanFilter(float dt, float processNoise, float sensorNoise, float estimatedError);

        float update(float measurement);

    private:
        float dt;
        float processNoise;
        float sensorNoise;
        float estimatedError;

        float x;
        float p;
    };
}