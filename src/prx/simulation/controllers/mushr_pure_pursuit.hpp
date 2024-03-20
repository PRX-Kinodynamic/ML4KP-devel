#pragma once
#include "prx/simulation/feedback_controller.hpp"
#include "prx/simulation/plants/mushr.hpp"

namespace prx
{
class mushr_pure_pursuit_t : public feedback_controller_t<trajectory_t>
{
private:
  system_ptr_t plant;

protected:
  double wheelbase, frequency, kp, ka, kb, time_window, time_ahead, goal_radius;
  unsigned _nearest_index, _window, _lookahead;
  int _reverse;

public:
  mushr_pure_pursuit_t(const system_ptr_t plant, param_loader params) : feedback_controller_t(plant, "mushr_pure_pursuit")
  {
    using prx::simulation_step;
    wheelbase = params["wheelbase"].as<double>();
    frequency = params["frequency"].as<double>();
    kp = params["k_p"].as<double>();
    ka = params["k_a"].as<double>();
    kb = params["k_b"].as<double>();
    time_window = params["time_window"].as<double>();
    time_ahead = params["time_ahead"].as<double>();
    goal_radius = params["goal_radius"].as<double>();

    _window = time_window / simulation_step;
    _lookahead = time_ahead / simulation_step;
  }

  using controller_t::compute_controls;
  void compute_controls() override
  {
    space_point_t current = get_state_space()->make_point();
    get_state_space()->copy_to(current);
    std::vector<double> control;
    get_control(current,control);
    get_control_space()->copy_from_vector(control);
    get_control_space()->enforce_bounds();
  }

  using controller_t::goal_reached;
  virtual bool goal_reached(const space_point_t& current_state) override
  {
    return goal_reached(current_state, df, goal_radius);
  }

  inline double angle_diff(const double a, const double b)
  {
    return std::atan2(std::sin(a - b), std::cos(a - b));
  }

  template <typename ControlPoint>
  void get_control(const space_point_t& current_state, ControlPoint& control)
  {
    control.resize(2);
    if (goal_reached(current_state))
    {
      _points = nullptr;
      _reverse = 0;
      control[0] = 0.;
      control[1] = 0.;
    }
    if (_points != nullptr)
    {
      unsigned window_size = std::min(_window + _nearest_index, _points->size());

      double dist = std::numeric_limits<double>::max();
      for (unsigned i = _nearest_index + 1; i < window_size; ++i)
      {
        double diff = space_t::euclidean_2d(_points->at(i), current_state, 0, 3);
        if (diff < dist)
        {
          dist = diff;
          _nearest_index = i;
        }
      }

      unsigned _lookahead_idx = _nearest_index + _lookahead;
      if (_lookahead_idx >= _points->size())
        _lookahead_idx = _points->size() - 1;

      const Eigen::Vector2d delta =
          _points->at(_lookahead_idx)->as<Eigen::VectorXd>().head(2) - current_state->as<Eigen::VectorXd>().head(2);
      const double theta = current_state->at(2);

      const double p = std::sqrt(std::pow(delta[0], 2) + std::pow(delta[1], 2));
      const double a = angle_diff(std::atan2(delta[1], delta[0]), theta);
      const int curr_reverse = (-prx::constants::pi / 2.0 < a && a <= prx::constants::pi / 2.0) ? 1 : -1;
      const double beta = prx::norm_angle_pi(angle_diff(-theta, a) + _points->at(_lookahead_idx)->at(2));
      if (_reverse == 0)
        _reverse = curr_reverse;
      
      const double v = kp * p;
      const double omega = (ka * a + kb * beta);

      control[0] = omega;
      control[1] = _reverse * v;
    }
    else
    {
      control[0] = 0.;
      control[1] = 0.;
    }
  }
};
}  // namespace prx