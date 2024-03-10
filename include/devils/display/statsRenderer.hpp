#pragma once
#include "../odom/odomPose.hpp"
#include "../odom/odomSource.hpp"
#include "../../pros/misc.hpp"
#include "displayUtils.hpp"
#include "renderer.hpp"
#include <cmath>
#include <string>

namespace devils
{
    /**
     * Renderer that displays the robot's current stats.
     */
    class StatsRenderer : public Renderer
    {
    public:
        ~StatsRenderer()
        {
            lv_obj_del(textObject);
        }

        void create(lv_obj_t *root)
        {
            // Label Object
            textObject = lv_label_create(root, NULL);
            {
                lv_label_set_align(textObject, LV_LABEL_ALIGN_LEFT);
                lv_label_set_recolor(textObject, true);
                lv_label_set_long_mode(textObject, LV_LABEL_LONG_BREAK);

                static lv_style_t textStyle;
                lv_style_copy(&textStyle, &lv_style_plain);
                textStyle.text.font = &lv_font_dejavu_10;
                textStyle.text.color = LV_COLOR_MAKE(0xff, 0xff, 0xff);
                textStyle.body.main_color = LV_COLOR_MAKE(0x2e, 0x96, 0xff);
                lv_obj_set_style(textObject, &textStyle);

                lv_obj_set_width(textObject, DisplayUtils::DISPLAY_WIDTH * 0.5);
                lv_obj_set_height(textObject, DisplayUtils::DISPLAY_HEIGHT - PADDING * 2);
                lv_obj_align(textObject, root, LV_ALIGN_IN_TOP_LEFT, PADDING, PADDING);
            }
        }

        void update()
        {
            // Get Stats
            double batteryPercent = pros::battery::get_capacity();
            bool isCompetition = pros::competition::is_connected();
            bool isAutonomous = pros::competition::is_autonomous();
            bool isDisabled = pros::competition::is_disabled();
            bool isSDInserted = pros::usd::is_installed();

            // Create Text
            std::stringstream stream;
            stream << "Battery: " << DisplayUtils::colorizeValue(batteryPercent / 100.0, std::to_string(batteryPercent) + "%");
            stream << "Competition: " << DisplayUtils::colorizeValue(isCompetition, isCompetition ? "Yes" : "No");
            stream << "Mode: " << DisplayUtils::colorizeValue(!isAutonomous, isAutonomous ? "Auto" : "Driver");
            stream << "Status: " << DisplayUtils::colorizeValue(!isDisabled, isDisabled ? "Disabled" : "Enabled");
            stream << "SD: " << DisplayUtils::colorizeValue(isSDInserted, isSDInserted ? "Installed" : "Removed");

            // Controller
            if (controller != nullptr && controller->is_connected())
            {
                int controllerBattery = controller->get_battery_level();
                stream << "\n";
                stream << "Controller: " << DisplayUtils::colorizeValue(controllerBattery / 100.0, std::to_string(controllerBattery) + "%");
            }

            // Odom
            if (odomSource != nullptr)
            {
                auto pose = odomSource->getPose();
                stream << "\n";
                stream << "X: " << (int)pose.x << " in\n";
                stream << "Y: " << (int)pose.y << " in\n";
                stream << "Rotation: " << (int)Units::radToDeg(pose.rotation) << "°\n";
            }

            // Additional Text
            stream << "\n";
            stream << additionalText;
            std::string text = stream.str();

            // Set Text
            lv_label_set_text(textObject, text.c_str());
        }

        /**
         * Uses a controller to display additional stats.
         * @param controller The master controller
         */
        void useController(pros::Controller *controller)
        {
            this->controller = controller;
        }

        /**
         * Uses an odom source to display additional stats.
         * @param odomSource The odom source
         */
        void useOdomSource(OdomSource *odomSource)
        {
            this->odomSource = odomSource;
        }

        /**
         * Appends additional text to the stats.
         * @param additionalText The additional text to append
         */
        void setAdditionalText(std::string additionalText)
        {
            this->additionalText = additionalText;
        }

    private:
        static constexpr int PADDING = 4;

        std::string additionalText = "";
        lv_obj_t *textObject;
        pros::Controller *controller;
        OdomSource *odomSource;
    };
}