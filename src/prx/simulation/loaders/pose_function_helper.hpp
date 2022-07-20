#pragma once

#include "prx/utilities/defs.hpp"

using namespace prx;

class position_function_t
{
public:
    virtual void init(const std::string& fd, const double& mult, const std::vector<double>& pos,
        double orn = 0.0)
    {
        std::cout << "Initializing position function: " << fd << " with mult: " << mult << " and pos: " << pos[0] << " " << pos[1] << " " << pos[2] << " and orn: " << orn << std::endl;
        function_descriptor = fd;
        multiplier = mult;
        initial_position = pos;
        initial_orientation = orn;
    }
    position_function_t()
    {
        function_descriptor = "";
        multiplier = 1;
        initial_position = std::vector<double>();
        initial_orientation = PRX_PI/2;
        last_reset_time = 0;
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
        else if (function_descriptor == "const")
        {
            while (last_reset_time > 0 && t >= last_reset_time) t -= last_reset_time;
            if (t < -PRX_EPSILON || (t >= last_reset_time && last_reset_time > 0)) 
                prx_throw_backtrace("Invalid time for position function: " + std::to_string(t) + " " + std::to_string(last_reset_time));
            result[0] += multiplier * std::cos(initial_orientation) * (t);
            result[1] += multiplier * std::sin(initial_orientation) * (t);
            // If the object goes outside the limits, then its position is reset.
            if (result[0] < -10 || result[0] > 10 || result[1] < -10 || result[1] > 10)
            {
                result[0] = initial_position[0];
                result[1] = initial_position[1];
                if (last_reset_time < PRX_EPSILON) last_reset_time = t;
            }
        }
        result.push_back(initial_orientation);
        return result;
    }
protected:
    std::vector<double> initial_position;
    double initial_orientation, last_reset_time;
    std::string function_descriptor;
    double multiplier;
};