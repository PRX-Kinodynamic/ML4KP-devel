#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
    class unicycle_so_t : public plant_t
    {
    public:
        unicycle_so_t(const std::string& path);

        virtual ~unicycle_so_t();

        virtual void propagate(const double simulation_step) override final;

        virtual void update_configuration() override;

        void set_state_space_bounds(const std::vector<double>& lower, const std::vector<double>& upper) override;

        virtual void compute_derivative() override final;
    protected:
    
            double x, y, theta, v, dx, dy, dtheta, dv, ddtheta, ddv;
    
            std::vector<double> lower_bound = {-11, -11, -3.15,-0.5,-0.5};
            std::vector<double> upper_bound = {11, 11, 3.15,0.5,0.5};
    };
}
PRX_REGISTER_SYSTEM(unicycle_so_t, SO_unicycle)