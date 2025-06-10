#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/system_group.hpp"
#include "prx/planning/world_model.hpp"

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/csv_reader.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/visualization/three_js_group.hpp"

#include "prx/factor_graphs/lie_groups/se3.hpp"
#include "prx/factor_graphs/lie_groups/so3.hpp"
#include "prx/factor_graphs/lie_groups/screw_axis.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
// #include "prx/factor_graphs/defs.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/JacobianFactor.h>

using SO3 = gtsam::SO3;
using TangentVec = Eigen::Vector3d;
using Integrator = prx::fg::lie_integrator_t<SO3, TangentVec>;

void random_tangent_vec(TangentVec& vec, const double& stddev)
{
  vec[0] = prx::gaussian_random(0.0, stddev);
  vec[1] = prx::gaussian_random(0.0, stddev);
  vec[2] = prx::gaussian_random(0.0, stddev);
}

void noisy_rotation(SO3& rot_hat, const SO3& rot, TangentVec& vec, const double& stddev)
{
  random_tangent_vec(vec, stddev);
  rot_hat = rot * SO3::Expmap(vec);
}

void rotation_with_noise(prx::param_loader& params)
{
  const int total_rotations{ params["total"].as<int>() };
  const double stddev{ params["stddev"].as<double>() };

  SO3 R;
  SO3 Rhat;
  TangentVec vec;

  std::ofstream ofs(params["out"].as<>());
  for (int i = 0; i < total_rotations; ++i)
  {
    R = prx::fg::random_SO3<SO3>();
    noisy_rotation(Rhat, R, vec, stddev);
    ofs << "R " << R << Rhat << "\n";
  }
  ofs.close();
}

void sphere_vectors_plot(const double epsilon = 0.1)
{
  using prx::split;
  using prx::utilities::convert_to;
  const Eigen::Vector3d v(0, 0, 1);  // Point at north pole of sphere 1
  const Eigen::Vector3d vp(Eigen::Vector3d(0, epsilon, 1).normalized());
  std::string input;

  Eigen::Matrix3d R{ Eigen::Matrix3d::Identity() };
  Eigen::Quaterniond q{ Eigen::Quaterniond::Identity() };
  Eigen::Vector4d vaux{ Eigen::Vector4d::Zero() };
  while (getline(std::cin, input))
  {
    const std::vector<std::string> line{ split<std::string>(input) };
    const std::string rotation_type{ line[0] };
    if (rotation_type == "R")
    {
      const std::size_t total_nums{ line.size() - 1 };
      prx_assert((total_nums % 9) == 0, "Wrong number of numbers");
      const std::size_t total_mats{ static_cast<std::size_t>(total_nums / 9.0) };

      for (int i = 0; i < total_mats; ++i)
      {
        for (int r = 0; r < 3; ++r)
        {
          for (int c = 0; c < 3; ++c)
          {
            const int idx{ 1 + (i * 9) + (3 * r + c) };
            // PRX_DBG_VARS(idx);
            R(r, c) = convert_to<double>(line[idx]);
          }
        }
        // PRX_DBG_VARS(R);
        const Eigen::Vector3d v0{ R * v };
        const Eigen::Vector3d v1{ R * vp - v0 };
        std::cout << v0.transpose() << " " << v1.transpose() << " ";
      }
      std::cout << "\n";
    }
    else if (rotation_type == "Q")
    {
      const std::size_t total_nums{ line.size() - 1 };
      prx_assert((total_nums % 4) == 0, "Wrong number of numbers");
      const std::size_t total_qs{ static_cast<std::size_t>(total_nums / 4.0) };

      for (int i = 0; i < total_qs; ++i)
      {
        for (int c = 0; c < 4; ++c)
        {
          const int idx{ 1 + (i * 4) + c };
          // PRX_DBG_VARS(idx);
          vaux[c] = convert_to<double>(line[idx]);
        }
        q.w() = vaux[0];
        q.x() = vaux[1];
        q.y() = vaux[2];
        q.z() = vaux[3];

        const Eigen::Vector3d v0{ q * v };
        const Eigen::Vector3d v1{ q * vp - v0 };
        std::cout << v0.transpose() << " " << v1.transpose() << " ";
      }
      std::cout << "\n";
    }
  }
}

void forward_propagation(prx::param_loader& params)
{
  SO3 x0;
  TangentVec xdot;
  const double dt{ params["dt"].as<double>() };
  const double stddev{ params["stddev"].as<double>() };
  const int total_rotations{ params["total"].as<int>() };

  std::ofstream ofs(params["out"].as<>());
  for (int i = 0; i < total_rotations; ++i)
  {
    x0 = prx::fg::random_SO3<SO3>();
    random_tangent_vec(xdot, stddev);
    const SO3 x1{ Integrator::integrate(x0, xdot, dt) };
    ofs << x0 << xdot << x1 << "\n";
  }
  ofs.close();
}

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params["epsilon"].set(0.1);

  params.add_opts(argc, argv);

  // prx::param_loader params(argc, argv);

  const std::string mode{ params["mode"].as<>() };

  if (mode == "rotation-with-noise")
  {
    rotation_with_noise(params);
  }
  else if (mode == "fwd-prop")
  {
    forward_propagation(params);
  }
  else if (mode == "to-plot")
  {
    const double epsilon{ params["epsilon"].as<double>() };

    sphere_vectors_plot(epsilon);
  }
  return 0;
}
