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

        void create(lv_obj_t *root)
        {
            // Get Path
            auto profilePoints = motionProfile->getProfilePoints();
            auto controlPoints = motionProfile->getControlPoints();

            // Calculate Offset
            double offsetX = DisplayUtils::DISPLAY_WIDTH / 2;
            double offsetY = DisplayUtils::DISPLAY_HEIGHT / 2;

            // Path
            static lv_obj_t *robotPath = lv_line_create(root, NULL);
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

            /*
            // Points
            static std::vector<lv_obj_t *> pointObjects;
            {
                static lv_style_t pointStyle;
                lv_style_copy(&pointStyle, &lv_style_plain);
                pointStyle.body.main_color = LV_COLOR_MAKE(0xff, 0x00, 0x00);
                pointStyle.body.grad_color = LV_COLOR_MAKE(0xff, 0x00, 0x00);
                pointStyle.body.radius = LV_RADIUS_CIRCLE;

                for (auto point : controlPoints)
                {
                    lv_obj_t *pointObject = lv_obj_create(root, NULL);
                    {
                        pointObjects.push_back(pointObject);

                        lv_obj_set_size(pointObject, 8, 8);
                        lv_obj_set_pos(pointObject, Units::metersToIn(point.x) + 50 - 4, Units::metersToIn(point.y) + 50 - 4);
                        lv_obj_set_style(pointObject, &pointStyle);
                    }
                }
            }
            */
        }

        void update() {}

    private:
        static constexpr float DT = 0.1;

        MotionProfile *motionProfile;
    };
}