#pragma once
#include "renderer.hpp"
#include "../utils/logger.hpp"
#include "../utils/units.hpp"
#include "../control/autoController.hpp"
#include <cmath>
#include <string>

namespace devils
{
    /**
     * Renderer that displays the controller status.
     */
    class ControlRenderer : public Renderer
    {
    public:
        /**
         * Creates a new control renderer with the autonomous controller
         * @param controller The autonomous controller to use.
         */
        ControlRenderer(AutoController *controller)
            : controller(controller)
        {
        }

        /**
         * Called when the renderer is destroyed.
         */
        ~ControlRenderer()
        {
            lv_obj_del(pointObject);
        }

        void create(lv_obj_t *root) override
        {
            pointObject = lv_obj_create(root, NULL);
            {
                lv_obj_set_size(pointObject, POINT_SIZE, POINT_SIZE);
                lv_obj_set_pos(pointObject, 0, 0);

                static lv_style_t pointStyle;
                lv_style_copy(&pointStyle, &lv_style_plain);
                pointStyle.body.main_color = LV_COLOR_MAKE(0xff, 0x00, 0x00);
                pointStyle.body.grad_color = LV_COLOR_MAKE(0xff, 0x00, 0x00);
                pointStyle.body.radius = LV_RADIUS_CIRCLE;
                lv_obj_set_style(pointObject, &pointStyle);
            }
        }

        void update() override
        {
            if (controller->isFinished())
                lv_obj_set_hidden(pointObject, true);

            auto point = controller->getTargetPose();
            lv_obj_set_pos(
                pointObject,
                point->x * DisplayUtils::PX_PER_IN + OFFSET_X,
                point->y * DisplayUtils::PX_PER_IN + OFFSET_Y);
        }

    private:
        static constexpr int POINT_SIZE = 12;
        static constexpr int OFFSET_X = (DisplayUtils::DISPLAY_WIDTH - POINT_SIZE) / 2;
        static constexpr int OFFSET_Y = (DisplayUtils::DISPLAY_HEIGHT - POINT_SIZE) / 2;

        lv_obj_t *pointObject;
        AutoController *controller;
    };
}