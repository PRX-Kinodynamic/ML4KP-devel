#pragma once 

#include "prx/simulation/plant.hpp"
#include "prx/simulation/controller.hpp"

namespace prx
{
    class mountain_car_t : public plant_t
    {
        public:
        mountain_car_t(const std::string& path);
        virtual ~mountain_car_t();

        virtual void propagate(const double simulation_step) override final;

        virtual void update_configuration() override final;

        protected:
        virtual void compute_derivative() override final;

        double x, xdot, xdotdot, u;
        double oldxdot;

        double m = 0.2;
        double g = -9.8;
        double k = 0.3;
    };
}

PRX_REGISTER_SYSTEM(mountain_car_t, mountain_car)
