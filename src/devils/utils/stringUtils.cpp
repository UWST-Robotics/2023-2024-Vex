#pragma once
#include "devils/utils/stringUtils.h"

std::vector<std::string> devils::StringUtils::split(std::string str, char delimiter)
{
    std::vector<std::string> result;
    std::string current = "";
    for (int i = 0; i < str.length(); i++)
    {
        if (str[i] == delimiter)
        {
            result.push_back(current);
            current = "";
        }
        else
        {
            current += str[i];
        }
    }
    result.push_back(current);
    return result;
}