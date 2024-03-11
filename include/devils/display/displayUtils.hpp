#pragma once

namespace devils
{
    /**
     * Utilities for rendering w/ LVGL.
     */
    class DisplayUtils
    {
    public:
        /**
         * Converts a value from 0-1 to a hex color.
         * Color scale is from red to yellow to green.
         * @param value The value to convert from 0-1
         * @return The hex color
         */
        static std::string colorScale(double value)
        {
            value = std::clamp(value, 0.0, 1.0);

            int r = 255;
            int g = 255;
            int b = 255;

            if (value < 0.5)
            {
                r = 255;
                g = value * 2 * 255;
            }
            else
            {
                r = (1 - value) * 2 * 255;
                g = 255;
            }

            return "#" + toHex(r) + toHex(g) + "00";
        }

        /**
         * Adds color to a string of text for LVGL.
         * @param text The text to colorize
         * @param colorHex The hex color to use (e.g. `#ff0000` for red)
         * @return The colorized text for LVGL. Format: `colorHex text#`
         */
        static std::string colorText(std::string text, std::string colorHex)
        {
            return colorHex + " " + text + "#";
        }

        /**
         * Converts a boolean to a hex color.
         * Red if false, green if true.
         * @param value The value to convert from 0-1
         * @return The hex color
         */
        static std::string colorScale(bool value)
        {
            return value ? "#00ff00" : "#ff0000";
        }

        /**
         * Colorizes a boolean value and adds a text label.
         * @param value The value to colorize
         * @param text The text to add
         * @return The colorized text for LVGL
         */
        static std::string colorizeValue(bool value, std::string text)
        {
            return colorText(text, colorScale(value)) + "\n";
        }

        /**
         * Colorizes a double value and adds a text label.
         * @param value The value to colorize
         * @param text The text to add
         * @return The colorized text for LVGL
         */
        static std::string colorizeValue(double value, std::string text)
        {
            return colorText(text, colorScale(value)) + "\n";
        }

        /**
         * Converts a value from 0-1 to a hex color.
         * @param value The value to convert from 0-1
         * @return The hex color
         */
        static std::string toHex(double value)
        {
            std::stringstream stream;
            stream << std::hex << (int)value;
            if (stream.str().length() == 1)
                return "0" + stream.str();
            return stream.str();
        }

        static constexpr double PX_PER_IN = 1.6;
        static constexpr int DISPLAY_WIDTH = 480;
        static constexpr int DISPLAY_HEIGHT = 240;
    };
}