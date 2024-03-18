#pragma once
#include "../utils/logger.hpp"
#include "units.hpp"
#include "pose.hpp"
#include <cmath>

// https://www.vexforum.com/t/eigen-integration-issue/61474/7
#undef __ARM_NEON__
#undef __ARM_NEON
#include "Eigen/Dense"

namespace devils
{

    /**
     * A class representing a 4-point perspective transform.
     */
    class Perspective
    {
    public:
        /**
         * Container for a point on the display.
         */
        struct DisplayPoint
        {
            /// @brief The x position of the point
            double x = 0;
            /// @brief The y position of the point
            double y = 0;

            /**
             * Prints the pose to a string
             * @return The pose as a string
             */
            const std::string toString()
            {
                return "Point(" + std::to_string(x) + ", " + std::to_string(y) + ")";
            }
        };

        /**
         * Generates a new Perspective transform.
         * @param from The starting points of the transform, in clockwise order from the top-left.
         * @param to The ending points of the transform, in clockwise order from the top-left.
         */
        Perspective(std::vector<DisplayPoint> from, std::vector<DisplayPoint> to)
        {
            if (from.size() != 4 || to.size() != 4)
            {
                Logger::error("Invalid number of points for perspective transform");
                return;
            }
            transformationMatrix = _calculateTransformationMatrix(from, to);
        }
        Perspective(Perspective &other)
        {
            transformationMatrix = other.transformationMatrix;
        }

        /**
         * Calculates the transformation matrix for the perspective transform.
         */
        Eigen::Matrix3d _calculateTransformationMatrix(std::vector<DisplayPoint> startingPoints, std::vector<DisplayPoint> endingPoints)
        {
            Eigen::Matrix<double, 8, 9> A;
            for (int i = 0; i < 4; i++)
            {
                A(i * 2, 0) = -startingPoints[i].x;
                A(i * 2, 1) = -startingPoints[i].y;
                A(i * 2, 2) = -1;
                A(i * 2, 3) = 0;
                A(i * 2, 4) = 0;
                A(i * 2, 5) = 0;
                A(i * 2, 6) = startingPoints[i].x * endingPoints[i].x;
                A(i * 2, 7) = startingPoints[i].y * endingPoints[i].x;
                A(i * 2, 8) = endingPoints[i].x;

                A(i * 2 + 1, 0) = 0;
                A(i * 2 + 1, 1) = 0;
                A(i * 2 + 1, 2) = 0;
                A(i * 2 + 1, 3) = -startingPoints[i].x;
                A(i * 2 + 1, 4) = -startingPoints[i].y;
                A(i * 2 + 1, 5) = -1;
                A(i * 2 + 1, 6) = startingPoints[i].x * endingPoints[i].y;
                A(i * 2 + 1, 7) = startingPoints[i].y * endingPoints[i].y;
                A(i * 2 + 1, 8) = endingPoints[i].y;
            }

            Eigen::JacobiSVD<Eigen::Matrix<double, 8, 9>> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix<double, 9, 1> h = svd.matrixV().col(8);
            Eigen::Matrix3d H;
            H << h(0), h(1), h(2),
                h(3), h(4), h(5),
                h(6), h(7), h(8);
            return H;
        }

        /**
         * Processes a point through the perspective transform.
         * @param point The point to transform.
         * @return The transformed point.
         */
        DisplayPoint pointToScreen(DisplayPoint &point)
        {
            Eigen::Vector3d p;
            p << point.x, point.y, 1;
            Eigen::Vector3d result = transformationMatrix * p;
            return {result(0) / result(2), result(1) / result(2)};
        }

        /**
         * Processes a point through the inverse perspective transform.
         * @param p The point to transform.
         * @return The transformed point.
         */
        DisplayPoint pointFromScreen(DisplayPoint &p)
        {
            Eigen::Vector3d point;
            point << p.x, p.y, 1;
            Eigen::Vector3d result = transformationMatrix.inverse() * point;
            return {result(0) / result(2), result(1) / result(2)};
        }

    private:
        Eigen::Matrix3d transformationMatrix;
    };
}