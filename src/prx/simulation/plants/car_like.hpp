#pragma once

#include "prx/simulation/plant.hpp"

namespace prx 
{
    /**
     * @brief <b> A second-order car-like system. </b>
     * 
     * @author Aravind Sivaramakrishnan, Edgar Granados, Noah Carver
     * 
     */
    class car_like_t : public plant_t
    {
        public:
        car_like_t(const std::string& path);

        virtual ~car_like_t();

        virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

		virtual void compute_derivative() override final;

        protected:
        double x,y,theta,phi,v,dx,dy,dtheta,dphi,dv;
        double L = 0.6;

        private:
        std::vector<double> lower_bound = {-11,-11,-3.15,-0.2,-PRX_PI/6};
        std::vector<double> upper_bound = { 11, 11, 3.15, 0.7, PRX_PI/6};
        friend system_factory_t;
    };
}
PRX_REGISTER_SYSTEM(car_like_t, car_like)