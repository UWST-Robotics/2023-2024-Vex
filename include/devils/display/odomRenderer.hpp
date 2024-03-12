#pragma once
#include "../odom/pose.hpp"
#include "../odom/odomSource.hpp"
#include "displayUtils.hpp"
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
        /**
         * Creates a new OdomRenderer
         * @param odomSource The odometry source to render
         */
        OdomRenderer(OdomSource *odomSource) : odomSource(odomSource)
        {
        }

        ~OdomRenderer()
        {
            lv_obj_del(robotObject);
            lv_obj_del(robotBGObject);
            lv_obj_del(robotPointer);
        }

        void create(lv_obj_t *root) override
        {
            // Robot Root
            robotObject = lv_obj_create(root, NULL);
            {
                lv_obj_set_size(robotObject, ROBOT_SIZE, ROBOT_SIZE);
                lv_obj_set_pos(robotObject, 0, 0);

                static lv_style_t transparentStyle;
                lv_style_copy(&transparentStyle, &lv_style_plain);
                transparentStyle.body.empty = 1;
                transparentStyle.body.border.width = 0;
                lv_obj_set_style(robotObject, &transparentStyle);
            }

            // Background
            robotBGObject = lv_obj_create(robotObject, NULL);
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
        }

        void update() override
        {
            // Pose
            auto pose = odomSource->getPose();

            // Object
            lv_obj_set_pos(
                robotObject,
                pose.x * DisplayUtils::PX_PER_IN + OFFSET_X,
                pose.y * DisplayUtils::PX_PER_IN + OFFSET_Y);

            // Arrow
            static lv_point_t linePoints[] = {
                {ROBOT_SIZE / 2, ROBOT_SIZE / 2},
                {0, 0}};
            linePoints[1].x = (short)(ROBOT_SIZE * cos(pose.rotation) * 0.5 + ROBOT_SIZE / 2);
            linePoints[1].y = (short)(ROBOT_SIZE * sin(pose.rotation) * 0.5 + ROBOT_SIZE / 2);
            lv_line_set_points(robotPointer, linePoints, 2);
        }

    private:
        static constexpr int ROBOT_SIZE = 15 * DisplayUtils::PX_PER_IN;
        static constexpr int OFFSET_X = (DisplayUtils::DISPLAY_WIDTH - ROBOT_SIZE) / 2;
        static constexpr int OFFSET_Y = (DisplayUtils::DISPLAY_HEIGHT - ROBOT_SIZE) / 2;

        OdomSource *odomSource;

        lv_obj_t *robotObject;
        lv_obj_t *robotBGObject;
        lv_obj_t *robotPointer;
    };
}