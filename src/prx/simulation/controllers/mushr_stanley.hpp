#pragma once
#include "prx/simulation/feedback_controller.hpp"
#include "prx/simulation/plants/mushr.hpp"

namespace prx
{
class mushr_stanley_t : public feedback_controller_t<trajectory_t>
{
private:
  system_ptr_t plant;

protected:
  double wheelbase, k_path, k_throttle, goal_radius;

  int discretization;
  std::vector<unsigned> indices;

public:
  mushr_stanley_t(const system_ptr_t plant, param_loader params)
    : feedback_controller_t(plant, params["name"].as<std::string>())
  {
    wheelbase = params["wheelbase"].as<double>();
    k_path = params["k_path"].as<double>();
    k_throttle = params["k_throttle"].as<double>();

    goal_radius = params["goal_radius"].as<double>();
    duration = 1.0 / params["frequency"].as<double>();
    discretization = std::floor(1.0 / (0.01 * params["frequency"].as<double>()));

    reset();
  }

  virtual void set_points(std::shared_ptr<trajectory_t> _traj) override
  {
    _points = _traj;
    indices.clear();
    for (int i = 0; i < _points->size(); i += discretization)
    {
      indices.push_back(i);
    }
    if (_points->size() % discretization != 0)
    {
      indices.push_back(_points->size() - 1);
    }
  }

  using controller_t::goal_reached;
  virtual bool goal_reached(const space_point_t& current_state) override
  {
    return goal_reached(current_state, df, goal_radius);
  }

  virtual void reset() override
  {
    _points = nullptr;
    indices.clear();
  }
  
  virtual void get_control(const space_point_t& current_state, Eigen::VectorXd& control) override
  {
    control.resize(2);
    // Check if the goal has been reached
    if (goal_reached(current_state))
    {
      reset();
      control[0] = 0.0;
      control[1] = 0.0;
    }
    if (_points != nullptr)
    {
      Eigen::Vector2d front_axle;
      front_axle << current_state->at(0) + wheelbase * std::cos(current_state->at(2)),
          current_state->at(1) + wheelbase * std::sin(current_state->at(2));

      std::vector<Eigen::Vector2d> diffs, projections;
      std::vector<double> l2s, dots, ts, dists;
      for (unsigned i = 1; i < indices.size(); ++i)
      {
        diffs.push_back(_points->at(indices[i])->as<Eigen::VectorXd>().head(2) -
                        _points->at(indices[i - 1])->as<Eigen::VectorXd>().head(2));
        l2s.push_back(diffs.back().norm());
      }
      for (unsigned i = 0; i < diffs.size(); ++i)
      {
        dots.push_back((front_axle - _points->at(indices[i])->as<Eigen::VectorXd>().head(2)).dot(diffs[i]) / l2s[i]);
        ts.push_back(std::clamp(dots[i] / l2s[i], 0.0, 1.0));
        projections.push_back(_points->at(indices[i])->as<Eigen::VectorXd>().head(2) + ts[i] * diffs[i]);
        dists.push_back((front_axle - projections[i]).norm());
      }

      double min_dist = std::numeric_limits<double>::max();
      unsigned nearest_index = 0;
      for (unsigned i = 0; i < dists.size(); ++i)
      {
        if (dists[i] < min_dist)
        {
          min_dist = dists[i];
          nearest_index = i;
        }
      }
      Eigen::Vector2d nearest_point = projections[nearest_index];
      Eigen::Vector2d vec_dist_nearest_point = front_axle - nearest_point;
      Eigen::Vector2d front_axle_vec_rotation;
      front_axle_vec_rotation << std::cos(current_state->at(2) - prx::constants::pi / 2.0),
          std::sin(current_state->at(2) - prx::constants::pi / 2.0);
      double cross = vec_dist_nearest_point.dot(front_axle_vec_rotation);

      double theta_line = _points->at(indices[nearest_index])->as<Eigen::VectorXd>()[2];
      double theta_e = prx::norm_angle_pi(theta_line - current_state->at(2));
      double theta_d = std::atan2(k_path * cross, current_state->at(3));

      double steering = theta_e + theta_d;
      double throttle = k_throttle * _points->at(indices[nearest_index])->as<Eigen::VectorXd>()[3];

      control[0] = steering;
      control[1] = throttle;
    }
    else
    {
      control[0] = 0.0;
      control[1] = 0.0;
    }
  }
};
}  // namespace prx
