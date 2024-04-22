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
     * Renderer that picks from a list of autonomous names
     */
    class AutoPickerRenderer : public Renderer
    {
    public:
        /**
         * Creates a new PathPickerRenderer
         * @param generatedPath The generated path to render
         */
        AutoPickerRenderer(std::initializer_list<std::string> autoNames) : autoNames(autoNames)
        {
            if (autoNames.size() == 0)
            {
                Logger::error("AutoPickerRenderer: No auto names provided");
                return;
            }
            currentAutoName = this->autoNames[0];
        }
        ~AutoPickerRenderer()
        {
            for (lv_obj_t *button : autoButtons)
                lv_obj_del(button);
        }

        void create(lv_obj_t *root) override
        {
            // Style
            static lv_style_t btnStyle;
            lv_style_copy(&btnStyle, &lv_style_plain);
            btnStyle.body.main_color = LV_COLOR_MAKE(0x00, 0x00, 0x00);
            btnStyle.body.grad_color = LV_COLOR_MAKE(0x00, 0x00, 0x00);
            btnStyle.body.radius = 1;
            btnStyle.body.border.color = LV_COLOR_MAKE(0xff, 0xff, 0xff);

            // Button
            for (int i = 0; i < autoNames.size(); i++)
            {
                autoButtons.push_back(_createButton(
                    root,
                    &btnStyle,
                    i,
                    10,
                    i * (BTN_HEIGHT + 10) + 10));
            }

            // Set the first button as selected
            lv_btn_set_state(autoButtons[0], LV_BTN_STATE_TGL_REL);
        }

        /**
         * Creates a button for the given auto name
         * @param root The root object
         * @param style The style to use
         * @param index The index of the auto name
         * @param offsetX The x offset
         * @param offsetY The y offset
         * @return The created button
         */
        lv_obj_t *_createButton(lv_obj_t *root, lv_style_t *style, int index, int offsetX, int offsetY)
        {
            // Button
            lv_obj_t *button = lv_btn_create(root, NULL);
            lv_obj_set_pos(button, DisplayUtils::DISPLAY_WIDTH - BTN_WIDTH - offsetX, offsetY);
            lv_obj_set_size(button, BTN_WIDTH, BTN_HEIGHT);
            lv_obj_set_style(button, style);
            lv_obj_set_free_num(button, index);

            // Label
            lv_obj_t *label = lv_label_create(button, NULL);
            lv_label_set_text(label, autoNames[index].c_str());

            return button;
        }

        /**
         * Called when a button is clicked
         * @param button The button that was clicked
         */
        void _onButtonClicked(lv_obj_t *button)
        {
            // Get the auto name from the button index
            int index = lv_obj_get_free_num(button);
            currentAutoName = autoNames[index];

            // Unhighlight all buttons
            for (lv_obj_t *btn : autoButtons)
                lv_btn_set_state(btn, LV_BTN_STATE_REL);

            // Highlight the clicked button
            lv_btn_set_state(button, LV_BTN_STATE_TGL_REL);
        }

        /**
         * Gets the currently selected auto name
         * @return The currently selected auto name
         */
        std::string getSelected()
        {
            return currentAutoName;
        }

    private:
        static constexpr int BTN_WIDTH = 110;
        static constexpr int BTN_HEIGHT = 48;

        std::vector<std::string> autoNames;
        std::vector<lv_obj_t *> autoButtons;
        std::string currentAutoName = "";
    };
}