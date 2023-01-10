#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"

namespace prx 
{
    class quadrotor_1d_t : public  ltv_t
    {
        public:
        quadrotor_1d_t(const std::string& path);
        virtual ~quadrotor_1d_t();

        virtual void propagate(const double simulation_step) override final;

        virtual void update_configuration() override;

        protected:
        virtual void compute_derivative() override final;

        double z, zdot, zdotdot, T;

        double g = 9.81;
        double m = 0.1;
    };
}
PRX_REGISTER_SYSTEM(quadrotor_1d_t, 1D_Quadrotor)