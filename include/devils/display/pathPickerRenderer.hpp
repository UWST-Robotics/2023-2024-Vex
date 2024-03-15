#pragma once
#include "../path/generatedPath.hpp"
#include "../utils/logger.hpp"
#include "displayUtils.hpp"
#include "renderer.hpp"
#include <cmath>
#include <string>

namespace devils
{
    /**
     * Renderer that picks from a list of paths
     */
    class PathPickerRenderer : public Renderer
    {
    public:
        /**
         * Creates a new PathPickerRenderer
         * @param generatedPath The generated path to render
         */
        PathPickerRenderer()
        {
        }

        ~PathPickerRenderer()
        {
            lv_obj_del(autoButton);
        }

        void create(lv_obj_t *root) override
        {
            // Calculate Offset
            double offsetX = DisplayUtils::DISPLAY_WIDTH / 2;
            double offsetY = DisplayUtils::DISPLAY_HEIGHT / 2;

            // Button
            autoButton = lv_btn_create(root, NULL);
            {
                // Style
                static lv_style_t btnStyle;
                lv_style_copy(&btnStyle, &lv_style_plain);
                btnStyle.body.main_color = LV_COLOR_MAKE(0x00, 0x00, 0x00);
                btnStyle.body.grad_color = LV_COLOR_MAKE(0x00, 0x00, 0x00);
                btnStyle.body.radius = 1;
                btnStyle.body.border.color = LV_COLOR_MAKE(0xff, 0xff, 0xff);
                lv_obj_set_style(autoButton, &btnStyle);
            }
        }

    private:
        static constexpr float DT = 0.1;

        std::vector<lv_point_t> linePointVector = {};
        lv_point_t *linePoints = nullptr;

        GeneratedPath *generatedPath;
        lv_obj_t *autoButton;
    };
}