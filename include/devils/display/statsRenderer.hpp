#pragma once
#include "../geometry/pose.hpp"
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

        void create(lv_obj_t *root) override
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

        void update() override
        {
            // Get Stats
            double batteryPercent = pros::battery::get_capacity();
            bool isCompetition = pros::competition::is_connected();
            bool isAutonomous = pros::competition::is_autonomous();
            bool isDisabled = pros::competition::is_disabled();
            bool isSDInserted = pros::usd::is_installed();

            // Create Text
            std::stringstream stream;
            stream << "Battery: " << DisplayUtils::colorizeValue(batteryPercent / 100.0, std::to_string((int)batteryPercent) + "%") << "\n";
            stream << "Competition: " << DisplayUtils::colorizeValue(isCompetition, isCompetition ? "Yes" : "No") << "\n";
            stream << "Mode: " << DisplayUtils::colorizeValue(!isAutonomous, isAutonomous ? "Auto" : "Driver") << "\n";
            stream << "Status: " << DisplayUtils::colorizeValue(!isDisabled, isDisabled ? "Disabled" : "Enabled") << "\n";
            stream << "SD: " << DisplayUtils::colorizeValue(isSDInserted, isSDInserted ? "Installed" : "Removed") << "\n";

            // Controller
            if (controller != nullptr && controller->is_connected())
            {
                int controllerBattery = controller->get_battery_level();
                stream << "\n";
                stream << "Controller: " << DisplayUtils::colorizeValue(controllerBattery / 100.0, std::to_string(controllerBattery) + "%") << "\n";
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

            // Auto Controller
            if (autoController != nullptr)
            {
                AutoState &autoState = autoController->getState();
                if (autoState.events->size() > 0)
                {
                    stream << "\n";
                    stream << "Events: \n";
                    for (PathEvent &event : *autoState.events)
                    {
                        stream << event.name;
                        if (event.params.size() > 0)
                            stream << DisplayUtils::colorText(" (" + event.params + ")", "#aaaaaa");
                        stream << "\n";
                    }
                }
            }

            // Chassis
            if (chassis != nullptr)
            {
                auto leftMotors = chassis->getLeftMotors().getMotors();
                auto rightMotors = chassis->getRightMotors().getMotors();

                stream << "\nL: ";
                for (auto &motor : leftMotors)
                {
                    double temp = motor.get()->getTemperature();
                    stream << DisplayUtils::colorizeValue(1 - (temp - 20.0) / 40.0, std::to_string((int)temp) + "C ");
                }
                stream << "\nR: ";
                for (auto &motor : rightMotors)
                {
                    double temp = motor.get()->getTemperature();
                    stream << DisplayUtils::colorizeValue(1 - (temp - 20.0) / 40.0, std::to_string((int)temp) + "C ");
                }
            }

            // Auto Picker
            if (autoPickerRenderer != nullptr)
            {
                stream << "\n";
                stream << "Auto: " << autoPickerRenderer->getSelected() << "\n";
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
         * Uses an auto controller to display additional stats.
         * @param controller The auto controller
         */
        void useAutoController(AutoController *controller)
        {
            this->autoController = controller;
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
         * Uses a tank chassis to display additional stats.
         * @param chassis The tank chassis
         */
        void useChassis(TankChassis *chassis)
        {
            this->chassis = chassis;
        }

        /**
         * Uses an auto picker renderer to display additional stats.
         * @param autoPickerRenderer The auto picker renderer
         */
        void useAutoPickerRenderer(AutoPickerRenderer *autoPickerRenderer)
        {
            this->autoPickerRenderer = autoPickerRenderer;
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
        lv_obj_t *textObject = nullptr;
        pros::Controller *controller = nullptr;
        AutoController *autoController = nullptr;
        OdomSource *odomSource = nullptr;
        TankChassis *chassis = nullptr;
        AutoPickerRenderer *autoPickerRenderer = nullptr;
    };
}