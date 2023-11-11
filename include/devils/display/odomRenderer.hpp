#pragma once
#include "../odom/pose.hpp"
#include "../odom/odomSource.hpp"
#include "renderer.hpp"
#include <cmath>
#include <string>

namespace devils
{
    /**
     * Renderer that displays the robot's position.
     */
    class OdomRenderer : public Renderer
    {
    public:
        OdomRenderer(OdomSource *odomSource)
        {
            this->odomSource = odomSource;
        }

        void create(lv_obj_t *root)
        {
            // Robot Root
            robotObject = lv_obj_create(root, NULL);
            {
                lv_obj_set_size(robotObject, 100, 100);
                lv_obj_set_pos(robotObject, 0, 0);

                static lv_style_t transparentStyle;
                lv_style_copy(&transparentStyle, &lv_style_plain);
                transparentStyle.body.empty = 1;
                transparentStyle.body.border.width = 0;
                lv_obj_set_style(robotObject, &transparentStyle);
            }

            // Background
            static lv_obj_t *robotBGObject = lv_obj_create(robotObject, NULL);
            {
                lv_obj_set_size(robotBGObject, 30, 30);
                lv_obj_align(robotBGObject, robotObject, LV_ALIGN_CENTER, 0, 0);

                static lv_style_t backgroundStyle;
                lv_style_copy(&backgroundStyle, &lv_style_plain);
                backgroundStyle.body.main_color = LV_COLOR_MAKE(0x2e, 0x96, 0xff);
                backgroundStyle.body.grad_color = LV_COLOR_MAKE(0x2e, 0x96, 0xff);
                lv_obj_set_style(robotBGObject, &backgroundStyle);
            }

            // Arrow
            robotPointer = lv_line_create(robotObject, NULL);
            {
                static lv_point_t linePoints[] = {{50, 50}, {50, 50}};
                lv_line_set_points(robotPointer, linePoints, 2);
                lv_line_set_auto_size(robotPointer, true);

                static lv_style_t pointerStyle;
                lv_style_copy(&pointerStyle, &lv_style_plain);
                pointerStyle.line.color = LV_COLOR_MAKE(0xff, 0xff, 0xff);
                pointerStyle.line.width = 3;
                lv_obj_set_style(robotPointer, &pointerStyle);
            }

            // Label
            robotLabel = lv_label_create(robotObject, NULL);
            {
                lv_label_set_text(robotLabel, "(0,0)");
                lv_label_set_long_mode(robotLabel, LV_LABEL_LONG_EXPAND);
                lv_obj_align(robotLabel, robotObject, LV_ALIGN_CENTER, 0, 0);
            }
        }

        void update()
        {
            // Pose
            Pose pose = odomSource->getPose();

            // Object
            lv_obj_set_pos(robotObject, pose.x, pose.y);

            // Arrow
            static lv_point_t linePoints[] = {
                {50, 50},
                {50, 50}};
            linePoints[1].x = 50 + 20 * cos(pose.rotation);
            linePoints[1].y = 50 + 20 * sin(pose.rotation);
            lv_line_set_points(robotPointer, linePoints, 2);
            Logger::warn(std::to_string(linePoints[1].x));

            // Label
            std::string text = "(" + std::to_string((int)pose.x) + "," + std::to_string((int)pose.y) + ")";
            lv_label_set_text(robotLabel, text.c_str());
            lv_label_set_align(robotLabel, LV_LABEL_ALIGN_CENTER);
        }

    private:
        OdomSource *odomSource;

        lv_obj_t *robotObject;
        lv_obj_t *robotPointer;
        lv_obj_t *robotLabel;
    };
}