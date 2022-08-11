#pragma once

#include "prx/utilities/defs.hpp"

using namespace prx;

static inline 
double triangle_wave(double t, double a, double p)
{
    return (2.0 * a / PRX_PI) * std::asin(std::sin(2.0 * PRX_PI * t/p));
}

static inline
double inverse_triangle_wave(double f, double a, double p)
{
    return (f * p) / (4.0 * a);
}

class position_function_t
{
public:
    virtual void init(const std::string& fd, const double& mult, const std::vector<double>& pos,
        double orn = 0.0)
    {
        // std::cout << "Initializing position function: " << fd << " with mult: " << mult << " and pos: " << pos[0] << " " << pos[1] << " " << pos[2] << " and orn: " << orn << std::endl;
        function_descriptor = fd;
        velocity = mult;
        initial_position = pos;
        initial_orientation = orn;
        if (function_descriptor == "triangle")
        {
            period_x = 40.0 / (velocity * std::cos(initial_orientation));
            period_y = 40.0 / (velocity * std::sin(initial_orientation));
            shift_x = inverse_triangle_wave(initial_position[0], 10.0, period_x);
            shift_y = inverse_triangle_wave(initial_position[1], 10.0, period_y);
        }
    }
    position_function_t()
    {
        function_descriptor = "";
        velocity = 1;
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
            result[1] = velocity * std::cos(0.33*t);
        }
        else if (function_descriptor == "sin")
        {
            result[1] = velocity * std::sin(0.33*t);
        }
        else if (function_descriptor == "triangle")
        {
            result[0] = triangle_wave(t - shift_x, 10.0, period_x);
            result[1] = triangle_wave(t - shift_y, 10.0, period_y);
            // while (last_reset_time > 0 && t >= last_reset_time) t -= last_reset_time;
            // if (t < -PRX_EPSILON || (t >= last_reset_time && last_reset_time > 0)) 
            //     prx_throw_backtrace("Invalid time for position function: " + std::to_string(t) + " " + std::to_string(last_reset_time));
            // result[0] += velocity * std::cos(initial_orientation) * (t);
            // result[1] += velocity * std::sin(initial_orientation) * (t);
            // // If the object goes outside the limits, then its position is reset.
            // if (result[0] < -10 || result[0] > 10 || result[1] < -10 || result[1] > 10)
            // {
            //     result[0] = initial_position[0];
            //     result[1] = initial_position[1];
            //     if (last_reset_time < PRX_EPSILON) last_reset_time = t;
            // }
        }
        result.push_back(initial_orientation);
        result.push_back(velocity);
        return result;
    }
protected:
    std::vector<double> initial_position;
    double initial_orientation, last_reset_time;
    // For triangle wave
    double period_x, period_y, shift_x, shift_y;
    std::string function_descriptor;
    double velocity;
};