#pragma once
#include "../path/motionProfile.hpp"
#include "../utils/logger.hpp"
#include "displayUtils.hpp"
#include "renderer.hpp"
#include <cmath>
#include <string>

namespace devils
{
    /**
     * Renderer that displays the robot's position.
     */
    class MotionRenderer : public Renderer
    {
    public:
        MotionRenderer(MotionProfile *motionProfile) : motionProfile(motionProfile)
        {
        }

        ~MotionRenderer()
        {
            lv_obj_del(robotPath);
        }

        void create(lv_obj_t *root) override
        {
            // Get Path
            auto profilePoints = motionProfile->getProfilePoints();
            auto controlPoints = motionProfile->getControlPoints();

            // Calculate Offset
            double offsetX = DisplayUtils::DISPLAY_WIDTH / 2;
            double offsetY = DisplayUtils::DISPLAY_HEIGHT / 2;

            // Path
            robotPath = lv_line_create(root, NULL);
            {
                // Convert path to vector of points
                static std::vector<lv_point_t> linePointVector;
                linePointVector.clear();
                for (int i = 0; i < profilePoints.size(); i++)
                {
                    linePointVector.push_back({(short)(profilePoints[i].x * DisplayUtils::PX_PER_IN + offsetX),
                                               (short)(profilePoints[i].y * DisplayUtils::PX_PER_IN + offsetY)});
                }

                // Create Line
                static lv_point_t *linePoints;
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

    private:
        static constexpr float DT = 0.1;

        MotionProfile *motionProfile;
        lv_obj_t *robotPath;
    };
}