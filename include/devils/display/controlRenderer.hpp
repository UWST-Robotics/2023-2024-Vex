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
        ControlRenderer(AutoController *controller)
            : controller(controller)
        {
        }

        void create(lv_obj_t *root)
        {
            pointObject = lv_obj_create(root, NULL);
            {
                lv_obj_set_size(pointObject, 8, 8);
                lv_obj_set_pos(pointObject, 0, 0);

                static lv_style_t pointStyle;
                lv_style_copy(&pointStyle, &lv_style_plain);
                pointStyle.body.main_color = LV_COLOR_MAKE(0x00, 0xff, 0x00);
                pointStyle.body.grad_color = LV_COLOR_MAKE(0x00, 0xff, 0x00);
                pointStyle.body.radius = LV_RADIUS_CIRCLE;
                lv_obj_set_style(pointObject, &pointStyle);
            }
        }

        void update()
        {
            auto point = controller->getCurrentProfilePoint();
            lv_obj_set_pos(pointObject, point.x + 50 - 4, point.y + 50 - 4);
        }

    private:
        lv_obj_t *pointObject;
        AutoController *controller;
    };
}