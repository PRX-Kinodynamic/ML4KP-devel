#pragma once 

#include "prx/simulation/plant.hpp"

namespace prx
{
    class trailer_car_t : public plant_t
    {
    public:
        trailer_car_t(const std::string& path);

        virtual ~trailer_car_t();

        virtual void propagate(const double simulation_step) override final;

        virtual void update_configuration() override;

        void set_state_space_bounds(const std::vector<double>& lower, const std::vector<double>& upper) override;

        virtual void compute_derivative() override final;
    
    protected:
        double x, y, theta0, theta1, dx, dy, dtheta0, dtheta1, v, phi;

        double L = 0.25;
        double d1 = 0.5;
    };
}
PRX_REGISTER_SYSTEM(trailer_car_t, trailer_car)
