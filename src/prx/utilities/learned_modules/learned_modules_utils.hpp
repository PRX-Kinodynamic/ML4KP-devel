#pragma once
#include "prx/utilities/defs.hpp"

std::vector<double> extract_state(const std::vector<double>& state, const std::vector<int>& indices)
{
    /*
        Extracts the state from the state vector.
    */
    std::vector<double> extracted_state;
    for (int i = 0; i < indices.size(); i++)
    {
        prx_assert(indices[i] < state.size(),"Index out of bounds!");
        extracted_state.push_back(state[indices[i]]);
    }
    return extracted_state;
}

std::vector<double> normalize_vector(const std::vector<double>& state, const std::vector<double>& lower_bounds, const std::vector<double>& upper_bounds)
{
    /*
        Normalizes the state to be between 0 and 1.
    */
    std::vector<double> normalized_state;
    for(int i = 0; i < state.size(); i++)
    {
        normalized_state.push_back((state[i] - lower_bounds[i])/(upper_bounds[i] - lower_bounds[i]));
    }
    return normalized_state;
}

std::vector<double> denormalize_control(const std::vector<double>& control, const std::vector<double>& lower_bounds, const std::vector<double>& upper_bounds)
{
    /*
        Denormalizes the control to be between the lower and upper bounds.
        Assumes the network outputs between -1 and 1.
    */
    std::vector<double> denormalized_control;
    for(int i = 0; i < control.size(); i++)
    {
        if (control[i] <= -1.0 || control[i] >= 1.0)
        {
            // PRX_DEBUG_PRINT
        }
        denormalized_control.push_back(0.5*(control[i] + 1.0)*(upper_bounds[i] - lower_bounds[i]) + lower_bounds[i]);
    }
    return denormalized_control;
}

std::vector<double> denormalize_state(const std::vector<double>& state, const std::vector<double>& lower_bounds, const std::vector<double>& upper_bounds)
{
    /*
        Denormalizes the state to be between the lower and upper bounds.
    */
    std::vector<double> denormalized_state;
    for(int i = 0; i < state.size(); i++)
    {
        denormalized_state.push_back(state[i]*(upper_bounds[i] - lower_bounds[i]) + lower_bounds[i]);
    }
    return denormalized_state;
}

double sigmoid(double x)
{
    /*
        Sigmoid function.
    */
    return 1.0/(1.0 + std::exp(-x));
}
