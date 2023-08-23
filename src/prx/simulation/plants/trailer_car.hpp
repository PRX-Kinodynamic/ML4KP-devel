#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
  /**
   * Trailer car (Eq. 13.19 from: http://lavalle.pl/planning/)
  */
class trailer_car_t : public plant_t
{
public:
  trailer_car_t(const std::string& path);
  virtual ~trailer_car_t();

  virtual void propagate(const double simulation_step) override final;

  virtual void update_configuration() override final;

  virtual void compute_derivative() override final;
protected:
  double x, y, theta0, theta1, v, phi, dx, dy, dtheta0, dtheta1;

  double L = 0.25;
  double d1 = 0.5;

  std::vector<double> lower_bound = {-11,-11, -prx::constants::pi, -prx::constants::pi};
  std::vector<double> upper_bound = { 11, 11,  prx::constants::pi,  prx::constants::pi};
};   
} // namespace prx
PRX_REGISTER_SYSTEM(trailer_car_t, trailer_car)