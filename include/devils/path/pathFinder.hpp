#pragma once
#include "../path/generatedPath.hpp"
#include "../odom/pose.hpp"
#include "../path/occupancyGrid.hpp"
#include "../path/pathGenerator.hpp"

namespace devils
{
    /**
     * Uses A* to calculate a path from a pose to a target pose
     */
    class PathFinder
    {
    public:
        /**
         * Generates a path to follow using the A* path planning algorithm
         * @param startPose - The starting pose of the robot
         * @param endPose - The ending pose of the robot
         * @param occupancyGrid - Data on where obstacles on the field are located
         * @return A path for the robot to follow in the form of a `GeneratedPath`
         */
        static GeneratedPath generatePath(Pose startPose,
                                          Pose endPose,
                                          OccupancyGrid &occupancyGrid)
        {
            Logger::info("Starting path finding...");

            // Get grid cells from pose
            GridPose startCell = _poseToGrid(startPose, occupancyGrid);
            GridPose endCell = _poseToGrid(endPose, occupancyGrid);

            // Init node stacks
            std::vector<AStarNode> allNodes;

            // Reserve space for all nodes
            // Also fixes bug where address of vector elements change
            allNodes.reserve(occupancyGrid.width * occupancyGrid.height);

            // Starting Node
            allNodes.push_back(startCell);
            auto startingNode = &allNodes.back();
            startingNode->x = startCell.x;
            startingNode->y = startCell.y;
            startingNode->gCost = 0;
            startingNode->fCost = startCell.getDistance(endCell);

            // Loop through unprocessed nodes
            while (true)
            {
                // Get Unprocessed Node w/ Lowest F-Cost
                AStarNode *_currentNode = nullptr;
                for (AStarNode &node : allNodes)
                    if (!node.isProcessed && (_currentNode == nullptr || node.fCost < _currentNode->fCost))
                        _currentNode = &node;

                // Break if no nodes are left
                if (_currentNode == nullptr)
                    break;

                // Mark node as processed
                AStarNode &currentNode = *_currentNode;
                currentNode.isProcessed = true;

                // Generate Path from Nodes
                if (currentNode == endCell)
                {
                    Logger::info("Found path!");
                    PathFile file = _nodeToPathFile(currentNode, occupancyGrid);
                    return PathGenerator::generateLinear(file);
                }

                // Iterate through neighbors
                auto neighbors = _getNeighborPoses(currentNode);
                for (auto &neighbor : neighbors)
                {
                    // Skip occupied cells & OOB cells
                    if (occupancyGrid.getOccupied(neighbor.x, neighbor.y))
                        continue;

                    // Search for existing node
                    AStarNode *existingNode = nullptr;
                    for (auto &node : allNodes)
                    {
                        if (node == neighbor)
                        {
                            existingNode = &node;
                            break;
                        }
                    }

                    // Skip if processed
                    if (existingNode != nullptr && existingNode->isProcessed)
                        continue;

                    // Calculate distance to node
                    int pathDistance = currentNode.getDistance(neighbor) + currentNode.gCost;

                    // Check if node should be updated
                    if (existingNode == nullptr || existingNode->gCost > pathDistance)
                    {

                        // If node doesn't exist, make it
                        if (existingNode == nullptr)
                        {
                            allNodes.push_back(neighbor);
                            existingNode = &allNodes.back();
                        }

                        // Update node target, gCost, and fCost
                        existingNode->parentNode = &currentNode;
                        existingNode->gCost = pathDistance;
                        existingNode->fCost = endCell.getDistance(neighbor) + pathDistance;
                    }
                }
            }

            // Path could not be solved
            Logger::error("PathFinder: Could not resolve path");
            Logger::info(startPose.toString() + " >>> " + endPose.toString());
        }

    protected:
        /// @brief A container for handling grid positions
        struct GridPose
        {
            /// @brief X position in cells
            int x = 0;
            /// @brief Y position in cells
            int y = 0;

            /**
             * Gets the approximate distance between each grid cell.
             * Uses `STRAIGHT_COST` and `DIAGONAL_COST` to calculate distance.
             * @param otherPose - The other pose to compare to
             * @return distance in cells
             */
            int getDistance(GridPose &otherPose)
            {
                // Delta position
                int deltaX = std::abs(x - otherPose.x);
                int deltaY = std::abs(y - otherPose.y);

                // Count straight/diagonal cells
                int straightCells = std::abs(deltaX - deltaY);
                int diagonalCells = std::max(deltaX, deltaY) - straightCells;

                // Amount * price = cost
                return straightCells * STRAIGHT_COST +
                       diagonalCells * DIAGONAL_COST;
            }

            /**
             * Compares two poses for equality
             * @param other The other pose
             * @return True if the poses are equal, false otherwise
             */
            bool operator==(const GridPose &other)
            {
                return x == other.x && y == other.y;
            }
        };

        /// @brief A container for each node for the A* algorithm
        struct AStarNode : public GridPose
        {
            AStarNode(GridPose p)
            {
                x = p.x;
                y = p.y;
            }

            /// @brief Distance from starting node
            int gCost = 0;

            /// @brief Sum of the starting distance and the ending distance
            int fCost = 0;

            /// @brief Whether the node has been processed and its neighbors have been evaluated
            bool isProcessed = false;

            /// @brief The node that this node originated from
            AStarNode *parentNode = nullptr;

            /**
             * Compares a node with a `GridPose` for equality
             * @param otherPose - The other pose to compare to
             * @return True if the poses are equal, false otherwise
             */
            bool operator==(const GridPose &otherPose)
            {
                return otherPose.x == x && otherPose.y == y;
            }
        };

        /**
         * Calculates a path file to the orgin by following the `parentNode` of each `AStarNode`
         * @param finalNode - The final node in the path
         * @param sourceGrid - The grid used for converting `AStarNode` to `Pose`
         * @returns A Generated Path using the `LinearGenerator`
         */
        template <typename T>
        static PathFile _nodeToPathFile(AStarNode &finalNode, Grid<T> sourceGrid)
        {
            // Initialize path file
            PathFile pathFile = PathFile();
            pathFile.version = 1;
            pathFile.points = std::vector<PathPoint>();

            // Iterate through node parents
            AStarNode *currentNode = &finalNode;
            while (currentNode != nullptr)
            {
                // Add pose to path
                auto pose = _gridToPose(*currentNode, sourceGrid);
                pathFile.points.insert(pathFile.points.begin(), PathPoint{pose});

                // Point to node's parent
                currentNode = currentNode->parentNode;
            }

            // Return path file
            return pathFile;
        }

        /**
         * Returns all 8 of the neighbors of a given pose
         * @param pose - Pose to evaluate
         * @return Vector of 8 neighboaring poses
         */
        static std::vector<GridPose> _getNeighborPoses(GridPose pose)
        {
            return {
                GridPose{pose.x + 1, pose.y - 1},
                GridPose{pose.x + 1, pose.y},
                GridPose{pose.x + 1, pose.y + 1},
                GridPose{pose.x, pose.y - 1},
                GridPose{pose.x, pose.y + 1},
                GridPose{pose.x - 1, pose.y - 1},
                GridPose{pose.x - 1, pose.y},
                GridPose{pose.x - 1, pose.y + 1}};
        }

        /**
         * Gets the closest unoccupied cell to the pose
         * @param pose - Pose to process
         * @param grid - The occupancy grid to cast onto.
         * @returns An unoccupied gridpose within the `grid` that closely matches `pose`
         */
        static GridPose _poseToGrid(Pose pose, OccupancyGrid &grid)
        {
            // Calculate Dimensions
            double cellWidth = FIELD_WIDTH / (double)grid.width;
            double cellHeight = FIELD_HEIGHT / (double)grid.height;

            // Field Pose relative to top-left
            double tlPoseX = pose.x + FIELD_WIDTH * 0.5;
            double tlPoseY = pose.y + FIELD_HEIGHT * 0.5;

            // Field Pose >> Cell Pose
            double xOrgin = tlPoseX / cellWidth;
            double yOrgin = tlPoseY / cellHeight;

            // Keep within bounds
            xOrgin = std::clamp((int)xOrgin, 0, grid.width);
            yOrgin = std::clamp((int)xOrgin, 0, grid.height);

            // Floor Index
            int x = xOrgin;
            int y = yOrgin;
            while (grid.getOccupied(x, y))
            {
                // TODO: Calculate closest unoccupied cell. Prioritize poses closer to decimal value
                Logger::error("Not yet implemented");
                break;
            }

            return GridPose{x, y};
        }

        /**
         * Converts a grid position to a pose
         * @param gridPose - Grid position to process
         * @param grid - Source grid to compare to
         * @returns A field pose that is in the center of the grid cell
         */
        template <typename T>
        static Pose _gridToPose(GridPose gridPose, Grid<T> grid)
        {
            // Calculate Dimensions
            double cellWidth = FIELD_WIDTH / (double)grid.width;
            double cellHeight = FIELD_HEIGHT / (double)grid.height;

            // GridPose >> Pose
            double poseX = gridPose.x * cellWidth;
            double poseY = gridPose.y * cellHeight;

            // Pose center in the grid cell
            poseX += cellWidth * 0.5;
            poseY += cellHeight * 0.5;

            // Pose Relative to Center
            poseX -= FIELD_WIDTH * 0.5;
            poseY -= FIELD_HEIGHT * 0.5;

            return Pose{poseX, poseY, 0};
        }

    private:
        static constexpr int FIELD_WIDTH = 144;  // in
        static constexpr int FIELD_HEIGHT = 144; // in
        static constexpr int STRAIGHT_COST = 10;
        static constexpr int DIAGONAL_COST = 14;
    };
}