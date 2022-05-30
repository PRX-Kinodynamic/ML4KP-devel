#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
    class unicycle_fo_t : public plant_t
    {
    public:
        unicycle_fo_t(const std::string& path);

        virtual ~unicycle_fo_t();

        virtual void propagate(const double simulation_step) override final;

        virtual void update_configuration() override;

        void set_state_space_bounds(const std::vector<double>& lower, const std::vector<double>& upper) override;

        virtual void compute_derivative() override final;
    protected:
    
            double x, y, theta, v, dx, dy, dtheta;
    
            std::vector<double> lower_bound = {-11, -11, -3.15};
            std::vector<double> upper_bound = {11, 11, 3.15};
    };
}
PRX_REGISTER_SYSTEM(unicycle_fo_t, FO_unicycle)