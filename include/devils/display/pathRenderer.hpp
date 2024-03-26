#pragma once
#include "../path/generatedPath.hpp"
#include "../utils/logger.hpp"
#include "displayUtils.hpp"
#include "renderer.hpp"
#include <cmath>
#include <string>

namespace devils
{
    /**
     * Renderer that displays the a robot's path.
     */
    class PathRenderer : public Renderer
    {
    public:
        /**
         * Creates a new PathRenderer
         * @param generatedPath The generated path to render
         */
        PathRenderer(GeneratedPath *generatedPath) : generatedPath(generatedPath)
        {
        }

        ~PathRenderer()
        {
            lv_obj_del(robotPath);
        }

        void create(lv_obj_t *root) override
        {
            rootObject = root;

            if (generatedPath == nullptr)
                return;

            // Get Path
            auto pathPoints = &generatedPath->pathPoints;
            auto controlPoints = &generatedPath->controlPoints;

            // Calculate Offset
            double offsetX = DisplayUtils::DISPLAY_WIDTH / 2;
            double offsetY = DisplayUtils::DISPLAY_HEIGHT / 2;

            // Path
            robotPath = lv_line_create(root, NULL);
            {
                // Convert path to vector of points
                linePointVector.clear();
                for (int i = 0; i < pathPoints->size(); i++)
                {
                    linePointVector.push_back({(short)(pathPoints->at(i).x * DisplayUtils::PX_PER_IN + offsetX),
                                               (short)(pathPoints->at(i).y * DisplayUtils::PX_PER_IN + offsetY)});
                }

                // Create Line
                linePoints = linePointVector.data();
                lv_line_set_points(robotPath, linePoints, linePointVector.size());

                // Style
                static lv_style_t pathStyle;
                lv_style_copy(&pathStyle, &lv_style_plain);
                pathStyle.line.color = LV_COLOR_MAKE(0xff, 0xff, 0xff);
                pathStyle.line.width = 2;
                lv_obj_set_style(robotPath, &pathStyle);
            }
        }

        /**
         * Switches the path to render
         * @param generatedPath The path to render
         */
        void setPath(GeneratedPath &generatedPath)
        {
            // Check if path is the same
            if (this->generatedPath == &generatedPath)
                return;

            // Update path
            this->generatedPath = &generatedPath;

            // Recreate path renderer
            if (robotPath != nullptr)
                lv_obj_del(robotPath);
            create(rootObject);
        }

    private:
        static constexpr float DT = 0.1;

        std::vector<lv_point_t> linePointVector = {};
        lv_point_t *linePoints = nullptr;

        GeneratedPath *generatedPath = nullptr;
        lv_obj_t *rootObject = nullptr;
        lv_obj_t *robotPath = nullptr;
    };
}