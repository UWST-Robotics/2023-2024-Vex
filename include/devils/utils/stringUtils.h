#pragma once
#include <string>
#include <vector>

namespace devils
{
    class StringUtils
    {
    public:
        /**
         * Splits a string by a delimiter.
         * @param str The string to split.
         * @param delimiter The delimiter to split by.
         */
        static std::vector<std::string> split(std::string str, char delimiter);
    };
}