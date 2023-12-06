#pragma once
#include "../path/motionProfile.hpp"
#include "renderer.hpp"
#include "../utils/logger.hpp"
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
        MotionRenderer(MotionProfile *motionProfile)
        {
            this->motionProfile = motionProfile;
        }

        void create(lv_obj_t *root)
        {
            // Generate
            motionProfile->generate();
            auto path = motionProfile->getPath();
            auto pathPoints = motionProfile->getPosePoints();

            // Path
            static lv_obj_t *robotPath = lv_line_create(root, NULL);
            {
                // Convert path to vector of points
                static std::vector<lv_point_t> linePointVector;
                linePointVector.clear();
                for (int i = 0; i < path.size(); i++)
                {
                    linePointVector.push_back({(short)(Units::metersToIn(path[i].vector.pose.x) + 50),
                                               (short)(Units::metersToIn(path[i].vector.pose.y) + 50)});
                }

                static lv_point_t *linePoints;
                linePoints = linePointVector.data();

                // Create line
                lv_line_set_points(robotPath, linePoints, linePointVector.size());
                lv_line_set_auto_size(robotPath, true);

                // Style
                static lv_style_t pathStyle;
                lv_style_copy(&pathStyle, &lv_style_plain);
                pathStyle.line.color = LV_COLOR_MAKE(0x00, 0x00, 0xff);
                pathStyle.line.width = 1;
                lv_obj_set_style(robotPath, &pathStyle);
            }

            // Points
            static std::vector<lv_obj_t *> pointObjects;
            {
                static lv_style_t pointStyle;
                lv_style_copy(&pointStyle, &lv_style_plain);
                pointStyle.body.main_color = LV_COLOR_MAKE(0xff, 0x00, 0x00);
                pointStyle.body.grad_color = LV_COLOR_MAKE(0xff, 0x00, 0x00);
                pointStyle.body.radius = LV_RADIUS_CIRCLE;

                for (auto point : pathPoints)
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
        }

        void update() {}

    private:
        static constexpr float DT = 0.1;

        MotionProfile *motionProfile;
    };
}