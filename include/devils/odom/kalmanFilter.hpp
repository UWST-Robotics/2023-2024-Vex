#pragma once

namespace devils
{
    class KalmanFilter
    {
    public:
        KalmanFilter(float dt, float processNoise, float sensorNoise, float estimatedError)
            : dt(dt), processNoise(processNoise), sensorNoise(sensorNoise), estimatedError(estimatedError)
        {
        }

        float update(float measurement)
        {
            // Predict
            x = x;
            p = p + processNoise;

            // Update
            float k = p / (p + sensorNoise);
            x = x + k * (measurement - x);
            p = (1 - k) * p + estimatedError;

            return x;
        }

    private:
        float dt;
        float processNoise;
        float sensorNoise;
        float estimatedError;

        float x;
        float p;
    };
}