#pragma once
#include <string>
#include <cmath>

namespace devils
{
    /**
     * Represents a pose in 2D space.
     */
    struct Pose
    {
        /// @brief The x position of the robot in inches
        double x = 0;
        /// @brief The y position of the robot in inches
        double y = 0;
        /// @brief The rotation of the robot in radians
        double rotation = 0;

        /**
         * Constructs a pose with all values set to 0
         */
        Pose() : x(0), y(0), rotation(0) {}

        /**
         * Constructs a pose with the given x and y
         * @param x The x position of the robot in inches
         * @param y The y position of the robot in inches
         */
        Pose(double x, double y) : x(x), y(y), rotation(0) {}

        /**
         * Constructs a pose with the given x, y, and rotation
         * @param x The x position of the robot in inches
         * @param y The y position of the robot in inches
         * @param rotation The rotation of the robot in radians
         */
        Pose(double x, double y, double rotation) : x(x), y(y), rotation(rotation) {}

        /**
         * Adds two poses together
         * @param other The other pose
         * @return The sum of the two poses
         */
        Pose operator+(const Pose &other)
        {
            return {x + other.x, y + other.y, rotation + other.rotation};
        }

        /**
         * Subtracts one pose from another
         * @param other The other pose
         * @return The difference of the two poses
         */
        Pose operator-(const Pose &other)
        {
            return {x - other.x, y - other.y, rotation - other.rotation};
        }

        /**
         * Multiplies a pose by a scalar
         * @param scalar The scalar to multiply by
         * @return The pose multiplied by the scalar
         */
        Pose operator*(const double &scalar)
        {
            return {x * scalar, y * scalar, rotation * scalar};
        }

        /**
         * Compares two poses for equality
         * @param other The other pose
         * @return True if the poses are equal, false otherwise
         */
        bool operator==(const Pose &other)
        {
            return x == other.x && y == other.y && rotation == other.rotation;
        }

        /**
         * Compares two poses for inequality
         * @param other The other pose
         * @return True if the poses are not equal, false otherwise
         */
        bool operator!=(const Pose &other)
        {
            return !(*this == other);
        }

        /**
         * Calculates the dot product of two poses
         * @param other The other pose
         * @return The dot product of the two poses
         */
        double dot(const Pose &other)
        {
            return x * other.x + y * other.y + rotation * other.rotation;
        }

        /**
         * Calculates the distance between two poses
         * @param other The other pose
         * @return The distance between the two poses
         */
        double distanceTo(const Pose &other)
        {
            return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
        }

        /**
         * Calculates the magnitude of the pose
         * @return The magnitude of the pose
         */
        double magnitude()
        {
            return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        }

        /**
         * Normalizes the pose
         * @return The normalized pose
         */
        Pose normalize()
        {
            double mag = magnitude();
            return {x / mag, y / mag, rotation};
        }

        /**
         * Prints the pose to a string
         * @return The pose as a string
         */
        const std::string toString()
        {
            return "Pose(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(rotation) + ")";
        }
    };
}