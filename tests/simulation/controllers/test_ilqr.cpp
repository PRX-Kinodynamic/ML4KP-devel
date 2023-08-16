#define BOOST_AUTO_TEST_MAIN lqr_controller_test

#include <boost/test/unit_test.hpp>
#include <string>
#include <iostream>
#include <ostream>
#include <fstream>

#include "prx/utilities/defs.hpp"
#include "prx/simulation/controllers/ilqr.hpp"
#include "prx/utilities/general/constants.hpp"

BOOST_AUTO_TEST_CASE(ilqr_pendulum)
{
  using ILQR = typename prx::simulation::iLQR_t<2, 1>;
  using DynamicFunction = typename ILQR::DynamicFunction;
  using VectorX = typename ILQR::VectorX;
  using VectorU = typename ILQR::VectorU;
  using MatrixQ = typename ILQR::MatrixQ;
  using MatrixR = typename ILQR::MatrixR;

  DynamicFunction xdot = [](const VectorX& vecX, const VectorU& vecU) {
    const double x{ vecX[0] };
    const double xdot{ vecX[1] };
    const double u{ vecU[0] };
    // PRX_DEBUG_VAR_3(x, xdot, u);

    const double m{ 1.0 };
    const double l{ 1.0 };
    const double g{ 9.8 };
    const double mu{ 0.01 };

    const double mll{ m * l * l };
    const double xdotdot{ (g / l) * std::sin(x) - (mu / mll) * xdot + (1.0 / mll) * u };
    return VectorX(xdot, xdotdot);
  };

  prx::simulation_step = 0.01;
  DynamicFunction f = [&](const VectorX& vecX, const VectorU& vecU) {
    return vecX + prx::simulation_step * xdot(vecX, vecU);
  };

  const MatrixQ Q{ MatrixQ::Identity() };
  const MatrixR R{ MatrixR::Identity() * 1e-4 };
  // const VectorX start_state{ VectorX(0.5, -1.0) };
  // const VectorX start_state{ VectorX(0.0, 0.0) };
  const VectorX start_state{ VectorX(3.14, 0) };
  // const VectorX start_state{ VectorX(0.5, 0.0) };
  const VectorX goal_state{ VectorX::Zero() };

  const double seconds{ 5.0 };
  std::size_t N{ static_cast<std::size_t>(seconds / prx::simulation_step) };
  std::vector<VectorX> traj{};
  std::vector<VectorU> ctrl{};

  std::vector<double> linear_traj{ prx::linspace(-0.2, 0.0, N) };
  // std::vector<double> linear_traj{ prx::linspace(0.5, 0.0, N) };
  for (int i = 0; i < N; ++i)
  {
    // traj.emplace_back(linear_traj[i] / 2.0, -linear_traj[i]);
    // traj.emplace_back(0.0, 0.0);
    traj.emplace_back(linear_traj[i], -linear_traj[i]);
    // traj.emplace_back(linear_traj[i], 0.0);
    std::cout << traj.back().transpose() << std::endl;
  }
  for (int i = 0; i < N - 1; ++i)
  {
    ctrl.emplace_back(VectorU::Zero());
  }

  std::ofstream ofs_map;
  ofs_map.open(prx::out_path + "/dbg_ilqr_res.txt", std::ofstream::trunc);

  // ilqr.update(traj, ctrl, VectorX::Zero(), 1e-6);

  VectorX state{ start_state };
  const std::size_t iters{ 100 };
  // for (int i = 0; i < N; ++i)
  // for (int i = 0; i < 1; ++i)
  // {
  PRX_DEBUG_VAR_1("------------------------------------");
  ILQR ilqr(f, Q, R, goal_state, Q);
  const VectorX x0{ traj[0] };
  // const VectorX dx0{ x0 - state };
  // traj.erase(traj.begin());
  // ctrl.erase(ctrl.begin());

  ilqr.update(ctrl, start_state, iters);
  // ilqr.to_file(ofs_map);
  ofs_map << "real ";
  ofs_map << state.transpose() << "\n";
  state = state + prx::simulation_step * xdot(state, ilqr.control(0));
  // }
  ofs_map << "real ";
  ofs_map << state.transpose() << "\n";

  // ofs_map.close();
  // for (int i = 0; i < N - 1; ++i)
  // {
  // std::cout << ilqr.state(i).transpose() << "\t";
  // std::cout << ilqr.control(i) << "\t";
  // std::cout << state.transpose() << "\n";
  // }
  // std::cout << ilqr.state(N - 1).transpose() << "\t";
  // std::cout << VectorU::Zero() << "\t";
  // std::cout << state.transpose() << "\n";
}