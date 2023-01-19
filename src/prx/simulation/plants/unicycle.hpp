#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
    class unicycle_t : public plant_t
    {
        public:
            unicycle_t(const std::string& path);
            virtual ~unicycle_t();
            virtual void propagate(const double simulation_step) override final;
            virtual void update_configuration() override;
            virtual void compute_stopping_maneuver(space_point_t, double&) override final;
            virtual void compute_derivative() override final;
        protected:
            double x,y,theta,v,omega,ul,ur,dx,dy;
        private:
            std::vector<double> lower_bound = {-11,-11,-3.15,-.7,-.7};
            std::vector<double> upper_bound = { 11, 11, 3.15, .7, .7};
            std::vector<double> ctrl_lower_bound = {-.2,-.2};
            std::vector<double> ctrl_upper_bound = {.2,.2};
            friend system_factory_t;
    };
}
PRX_REGISTER_SYSTEM(unicycle_t, unicycle)