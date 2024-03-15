#pragma once
#include "pros/gps.hpp"
#include "pose.hpp"
#include "../utils/logger.hpp"
#include "odomSource.hpp"
#include "../utils/units.hpp"
#include "../utils/runnable.hpp"

namespace devils
{
    /**
     * Represents a set of odometry sources merged into one using the complementary filter.
     */
    class ComplementaryFilterOdom : public OdomSource
    {
    public:
        /**
         * Creates a new merged odometry source
         * @param odomSources The odometry sources to merge
         * @param weights The weights to apply to each odometry source. Values should be between 0 and 1 and sum to 1.
         */
        ComplementaryFilterOdom(std::initializer_list<OdomSource *> odomSources,
                                std::initializer_list<double> weights)
            : odomSources(odomSources),
              weights(weights)
        {
            // Compare Weight and Source Sizes
            if (odomSources.size() != weights.size())
                throw std::invalid_argument("The number of odometry sources and weights must be the same");

            // Check Sum of Weights
            double sum = _getSumOfWeights();
            if (sum != 1)
                throw std::invalid_argument("Weights must sum to 1, currently sum to " + std::to_string(sum));

            // Check each weight
            for (int i = 0; i < weights.size(); i++)
                if (this->weights[i] < 0 || this->weights[i] > 1)
                    throw std::invalid_argument("Weight " + std::to_string(i) + " is not between 0 and 1");
        }

        /**
         * Gets the sum of the weights of each odometry source
         * @return The sum of all the weights
         */
        double _getSumOfWeights()
        {
            double sum = 0;
            for (int i = 0; i < weights.size(); i++)
                sum += weights[i];
            return sum;
        }

        /**
         * Updates the odometry with the latest data from the sources
         */
        void update()
        {
            // Get the current pose from each source
            std::vector<Pose> poses;
            for (int i = 0; i < odomSources.size(); i++)
                poses.push_back(odomSources[i]->getPose());

            // Merge the poses
            currentPose = Pose(0, 0, 0);
            for (int i = 0; i < poses.size(); i++)
                currentPose = currentPose + (poses[i] * weights[i]);

            // Reset the sources
            for (int i = 0; i < odomSources.size(); i++)
                odomSources[i]->setPose(currentPose);
        }

        /**
         * Gets the current pose of the robot
         * @return The current pose of the robot
         */
        Pose &getPose() override
        {
            return currentPose;
        }

        /**
         * Sets the current pose of the robot
         * @param pose The pose to set the robot to
         */
        void setPose(Pose &pose) override
        {
            currentPose = pose;
            for (int i = 0; i < odomSources.size(); i++)
                odomSources[i]->setPose(pose);
        }

    private:
        std::vector<OdomSource *> odomSources;
        std::vector<double> weights;
        Pose currentPose = Pose(0, 0, 0);
    };
}