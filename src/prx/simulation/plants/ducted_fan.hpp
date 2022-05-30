#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
    class ducted_fan_t : public plant_t
    {
    public:
        ducted_fan_t(const std::string& path);

        virtual ~ducted_fan_t();

        virtual void propagate(const double simulation_step) override final;

        virtual void update_configuration() override;

        virtual void compute_derivative() override final;
    protected:
    
            double x, y, theta, v, dx, dy, dtheta, ddx, ddy, ddtheta, u1, u2;
    
            std::vector<double> lower_bound = {-11, -11, -3.15, -5, -5, -2*PRX_PI};
            std::vector<double> upper_bound = {11, 11, 3.15, 5, 5, 2*PRX_PI};

            double g = 0.28;
            double m = 11.2;
            double I = 0.0462;
            double r = 0.156;
            double d = 0.1;
    };
}
PRX_REGISTER_SYSTEM(ducted_fan_t, ducted_fan)