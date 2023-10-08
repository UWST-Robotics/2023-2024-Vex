#pragma once
#include "devils/utils/stringUtils.h"

std::vector<std::string> splitString(std::string inputText, char delimiter)
{
    std::vector<std::string> result;
    std::string buffer = "";
    // Iterate through characters
    for (int i = 0; i < inputText.length(); i++)
    {
        // Flush current string if matches delimiter
        if (inputText[i] == delimiter)
        {
            result.push_back(buffer);
            buffer = "";
        }
        // Otherwise, add to buffer
        else
        {
            buffer += inputText[i];
        }
    }
    // Flush last string
    result.push_back(buffer);
    return result;
}