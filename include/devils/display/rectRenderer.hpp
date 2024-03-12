#pragma once
#include "renderer.hpp"
#include "../utils/logger.hpp"
#include "../utils/units.hpp"
#include "../utils/rect.hpp"
#include "displayUtils.hpp"
#include <cmath>
#include <string>

namespace devils
{
    /**
     * Renderer that displays a rectangle.
     */
    class RectRenderer : public Renderer
    {
    public:
        /**
         * Creates a new RectRenderer
         * @param rectangle The rectangle to render
         */
        RectRenderer(Rect *rectangle) : rect(rectangle)
        {
        }

        ~RectRenderer()
        {
            lv_obj_del(rectObject);
        }

        void create(lv_obj_t *root) override
        {
            // Calculate Dimensions
            double rectWidth = rect->width * DisplayUtils::PX_PER_IN;
            double rectHeight = rect->height * DisplayUtils::PX_PER_IN;

            double offsetX = DisplayUtils::DISPLAY_WIDTH / 2;
            double offsetY = DisplayUtils::DISPLAY_HEIGHT / 2;

            double rectX = rect->x * DisplayUtils::PX_PER_IN + offsetX;
            double rectY = rect->y * DisplayUtils::PX_PER_IN + offsetY;

            // Bounding Rect
            rectObject = lv_obj_create(root, NULL);
            {
                lv_obj_set_size(rectObject, rectWidth, rectHeight);
                lv_obj_set_pos(rectObject, rectX, rectY);

                static lv_style_t rectStyle;
                lv_style_copy(&rectStyle, &lv_style_plain);
                rectStyle.body.main_color = LV_COLOR_MAKE(0x44, 0x44, 0xff);
                rectStyle.body.grad_color = LV_COLOR_MAKE(0x44, 0x44, 0xff);
                rectStyle.body.border.color = LV_COLOR_MAKE(0x44, 0x44, 0xff);
                rectStyle.body.opa = 20;
                rectStyle.body.border.width = 2;
                rectStyle.body.radius = 4;
                lv_obj_set_style(rectObject, &rectStyle);
            }
        }

    private:
        Rect *rect;
        lv_obj_t *rectObject;
    };
}