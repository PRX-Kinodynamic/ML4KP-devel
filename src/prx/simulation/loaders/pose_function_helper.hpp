#pragma once

#include "prx/utilities/defs.hpp"

using namespace prx;

class position_function_t
{
public:
    virtual void init(const std::string& fd, const double& mult, const std::vector<double>& pos)
    {
        function_descriptor = fd;
        multiplier = mult;
        initial_position = pos;
    }
    position_function_t()
    {
        function_descriptor = "";
        multiplier = 1;
        initial_position = std::vector<double>();
    }
    virtual ~position_function_t(){}
    virtual std::vector<double> operator()(double t)
    {
        std::vector<double> result = initial_position;
        if (function_descriptor == "cos")
        {
            result[1] = multiplier * std::cos(0.33*t);
        }
        else if (function_descriptor == "sin")
        {
            result[1] = multiplier * std::sin(0.33*t);
        }
        return result;

    }
protected:
    std::vector<double> initial_position;
    std::string function_descriptor;
    double multiplier;
};