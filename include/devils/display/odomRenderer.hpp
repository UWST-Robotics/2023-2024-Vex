#pragma once
#include "../odom/odomPose.hpp"
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

        ~OdomRenderer()
        {
            lv_obj_del(robotObject);
            lv_obj_del(robotPointer);
            lv_obj_del(robotLabel);
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
                lv_obj_set_size(robotBGObject, ROBOT_SIZE, ROBOT_SIZE);
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
                static lv_point_t linePoints[] = {{0, 0}, {0, 0}};
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
                lv_obj_align(robotLabel, robotObject, LV_ALIGN_IN_TOP_MID, 0, 0);

                static lv_style_t labelStyle;
                lv_style_copy(&labelStyle, &lv_style_plain);
                labelStyle.text.color = LV_COLOR_MAKE(0xff, 0xff, 0xff);
                lv_obj_set_style(robotLabel, &labelStyle);
            }
        }

        void update()
        {
            // Pose
            OdomPose pose = odomSource->getPose();

            // Object
            lv_obj_set_pos(robotObject, pose.x, pose.y);

            // Arrow
            static lv_point_t linePoints[] = {
                {50, 50},
                {0, 0}};
            linePoints[1].x = 50 + ROBOT_SIZE * cos(pose.rotation);
            linePoints[1].y = 50 + ROBOT_SIZE * sin(pose.rotation);
            lv_line_set_points(robotPointer, linePoints, 2);

            // Label
            std::string text = "(" + std::to_string((int)pose.x) + "," + std::to_string((int)pose.y) + ")";
            lv_label_set_text(robotLabel, text.c_str());
            lv_label_set_align(robotLabel, LV_LABEL_ALIGN_CENTER);
        }

    private:
        static constexpr int ROBOT_SIZE = 12;

        OdomSource *odomSource;

        lv_obj_t *robotObject;
        lv_obj_t *robotPointer;
        lv_obj_t *robotLabel;
    };
}