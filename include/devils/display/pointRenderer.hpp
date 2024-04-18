#pragma once
#include "../geometry/pose.hpp"
#include "../odom/odomSource.hpp"
#include "displayUtils.hpp"
#include "renderer.hpp"
#include <cmath>
#include <string>

namespace devils
{
    /**
     * Renderer that displays a `DisplayPoint` object.
     */
    class PointRenderer : public Renderer
    {
    public:
        /**
         * Creates a new DisplayPointRenderer
         * @param r The red value of the cursor
         * @param g The green value of the cursor
         * @param b The blue value of the cursor
         */
        PointRenderer(u_char r, u_char g, u_char b) : odomSource(odomSource)
        {
            cursorColor = LV_COLOR_MAKE(r, g, b);
        }

        ~PointRenderer()
        {
            lv_obj_del(cursorObject);
        }

        void create(lv_obj_t *root) override
        {
            // Cursor
            cursorObject = lv_obj_create(root, NULL);
            {
                lv_obj_set_size(cursorObject, CURSOR_SIZE, CURSOR_SIZE);
                lv_obj_set_pos(cursorObject, 0, 0);

                static lv_style_t cursorStyle;
                lv_style_copy(&cursorStyle, &lv_style_plain);
                cursorStyle.body.grad_color = cursorColor;
                cursorStyle.body.main_color = cursorColor;
                lv_obj_set_style(cursorObject, &cursorStyle);
            }
        }

        void setPoint(Vector2 &point)
        {
            // Update Cursor Position
            lv_obj_set_pos(
                cursorObject,
                point.x * DISPLAY_SCALE + OFFSET_X,
                point.y * DISPLAY_SCALE + OFFSET_Y);
        }

    private:
        static constexpr int CURSOR_SIZE = 15 * DisplayUtils::PX_PER_IN;
        static constexpr int DISPLAY_SCALE = 1;
        static constexpr int OFFSET_X = (DisplayUtils::DISPLAY_WIDTH - CURSOR_SIZE) / 2;
        static constexpr int OFFSET_Y = (DisplayUtils::DISPLAY_HEIGHT - CURSOR_SIZE) / 2;

        OdomSource *odomSource;

        lv_color_t cursorColor;
        lv_obj_t *cursorObject;
    };
}