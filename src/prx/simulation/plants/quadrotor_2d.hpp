#pragma once

#include "prx/simulation/plant.hpp"

namespace prx 
{
    class quadrotor_2d_t : public plant_t 
    {
        public:
        quadrotor_2d_t(const std::string& path);
        virtual ~quadrotor_2d_t();

        virtual void propagate(const double simulation_step) override final;

        virtual void update_configuration() override;

        protected:
        virtual void compute_derivative() override final;

        double x, z, theta, vx, vz, thetadot,
        xdot, zdot, thetadotdot, vxdot, vzdot,
        T1, T2;

        double g = 9.81;
        double m = 0.486;
        double J = 0.00383;
        double l = 0.25; 
    };
}
PRX_REGISTER_SYSTEM(quadrotor_2d_t, 2D_Quadrotor)