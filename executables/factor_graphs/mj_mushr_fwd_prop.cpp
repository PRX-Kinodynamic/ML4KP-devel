#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/type_conversions.hpp"
#include "prx/factor_graphs/lie_groups/se3.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt_star.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/collision_checking/collision_group.hpp"

#include "prx/simulation/plants/first_order_free_body.hpp"
#include "prx/factor_graphs/plants/learned_pendulum.hpp"
#include "prx/factor_graphs/plants/learned_mushr.hpp"

// using State = Eigen::Vector<double, 2>;
// using Control = Eigen::Vector<double, 1>;
// using LearnedPendulum = prx::fg::learned_pendulum_t;
using CsvReader = prx::utilities::csv_reader_t;

bool read_trajectory(CsvReader& reader, prx::plan_t& plan, prx::trajectory_t& traj, const std::string plant_name,
                     std::vector<Eigen::Vector3d>& accel)
{
  using prx::utilities::convert_to;
  //  xi[0], xi[1], ui, dt, Gt, accel

  prx::fg::SE2_t x0;
  Eigen::VectorXd state;
  Eigen::VectorXd control;
  double dt{ 0 };

  bool trajs_eof{ true };
  while (reader.has_next_line())
  {
    trajs_eof = false;
    auto line = reader.next_line();
    if (line.size() == 0)
      break;

    if (plant_name == "learned_pendulum")
    {
      const double x{ convert_to<double>(line[0]) };
      const double xd{ convert_to<double>(line[1]) };

      const double u{ convert_to<double>(line[2]) };

      dt = convert_to<double>(line[3]);

      state = Eigen::Vector2d(x, xd);
      control = Eigen::Vector<double, 1>(u);
    }
    else if (plant_name == "learned_mushr")
    {
      const double x{ convert_to<double>(line[1]) };
      const double y{ convert_to<double>(line[2]) };
      const double th{ convert_to<double>(line[3]) };

      const double xd{ convert_to<double>(line[4]) };
      const double yd{ convert_to<double>(line[5]) };
      const double thd{ convert_to<double>(line[6]) };

      const double xdd{ convert_to<double>(line[7]) };
      const double ydd{ convert_to<double>(line[8]) };
      const double thdd{ convert_to<double>(line[9]) };

      const double u0{ convert_to<double>(line[10]) };
      const double u1{ convert_to<double>(line[11]) };

      state = Eigen::Vector<double, 6>(x, y, th, xd, yd, thd);
      // state = Eigen::Vector<double, 6>::Zero();
      control = Eigen::Vector<double, 2>(u0, u1);

      accel.push_back(Eigen::Vector3d(xdd, ydd, thdd));
      if (dt == 0)
      {
        x0 = prx::fg::SE2_t(x, y, th).inverse();
        // x0 = prx::euler_to_rotation<Eigen::Matrix3d>(Eigen::Vector<double, 1>(th), "Z");
        // x0(0, 2) = x;
        // x0(1, 2) = y;
        // PRX_DBG_VARS(x0);
        // x0 = x0.inverse().eval();
        // PRX_DBG_VARS(x0);
      }

      // auto xp = x0 * prx::fg::SE2_t(state.head(3)).matrix();
      // auto xp = x0 * prx::fg::SE2_t(x, y, th);
      // state[0] = xp[0];
      // state[1] = xp[1];
      // state[2] = xp[2];
      // PRX_DBG_VARS(xp, state.transpose());

      dt = convert_to<double>(line[0]);
    }

    traj.push_back(state);
    plan.copy_onto_back(control, dt);
  }
  return trajs_eof;
}
int main(int argc, char* argv[])
{
  prx::param_loader params(argc, argv);
  const std::string plant_file{ params["plant"].as<>() };
  params["plant"].add_file(plant_file);
  const std::string plant_name{ params["plant/name"].as<>() };  // const std::string torch_pt{ params["model"].as<>() };
  prx::simulation_step = params["/plant/simulation_step"].as<double>();

  if (params.exists("torch_file"))
  {
    params["plant/torch_file"] = params["torch_file"];
  }
  prx::system_ptr_t plant{ prx::system_factory_t::create_system(plant_name, plant_name) };
  // PRX_DBG_VARS(plant)
  // std::shared_ptr<LearnedPendulum> plant{ std::make_shared<LearnedPendulum>(plant_name, torch_pt) };
  prx_assert(plant != nullptr, "Plant is nullptr!");
  plant->init(params["plant"]);

  prx::world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  prx::simulation_context context{ world_model.get_context("context") };

  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };

  const std::size_t max_len = params["max_traj_len"].as<int>();
  CsvReader reader(params["filename"].as<>(), ' ');
  std::ofstream ofs(params["out"].as<>().c_str());
  const int tot_trajs{ params["total_trajs"].as<int>() };
  // const double inf{ std::numeric_limits<double>::infinity() };
  // ss->set_bounds({ -PRX_PI, -inf }, { PRX_PI, inf });

  std::vector<Eigen::Vector3d> accel;
  for (int i = 0; i < tot_trajs; ++i)
  {
    prx::plan_t plan(cs);
    prx::trajectory_t traj_in(ss);
    prx::trajectory_t traj_out(ss);

    const bool eof{ read_trajectory(reader, plan, traj_in, plant_name, accel) };
    if (eof)  // No more data
    {
      break;
    }
    if (traj_in.size() == 0)  // More than 1 empty lines between trajs
    {
      i--;
      continue;
    }
    // PRX_DBG_VARS(plan);

    prx::fg::SE2_t xi{ Vec(traj_in.front()).head(3) };
    prx::fg::SE2_t x_cte(0, 0, prx::constants::pi / 2.0);
    using LieIntegrator = prx::fg::lie_integrator_t<prx::fg::SE2_t, Eigen::Vector<double, 3>, double>;
    using EulerIntegrator =
        prx::fg::euler_integration_factor_t<Eigen::Vector<double, 3>, Eigen::Vector<double, 3>, double>;

    PRX_DBG_VARS(x_cte.matrix());
    ofs << traj_in.front() << " " << xi << " 0 0 0" << "\n";
    // for (auto iter = traj_in.begin() + 1; iter != traj_in.end(); iter++)
    // for (auto ti : traj_in)
    PRX_DBG_VARS(traj_in.size(), accel.size());
    int tot = accel.size();
    Eigen::Vector3d vel{ Eigen::Vector3d::Zero() };
    for (int i = 0; i < tot; ++i)
    {
      // auto ti = traj_in[i];
      // auto ti = *iter;
      // prx::fg::SE2_t xt{ Vec(ti).head(3) };
      // const Eigen::Vector3d vel{ prx::fg::lie_operators::right_minus(xt, xi) / prx::simulation_step };

      // const Eigen::Vector3d vel{ Vec(ti).tail(3) };
      // const Eigen::Vector3d velp{ x_cte.matrix() * vel };
      auto acc = accel[i];
      vel = EulerIntegrator::integrate(vel, acc, prx::simulation_step);
      xi = LieIntegrator::integrate(xi, vel, prx::simulation_step);
      // xi = LieIntegrator::integrate(xi, vel, prx::simulation_step);
      ofs << xi << " " << vel.transpose() << " " << acc.transpose() << "\n";
      // ofs << ti << " " << xi << " " << velp.transpose() << "\n";
    }
    // sg->propagate(traj_in.front(), plan, traj_out);

    // The last state of trajin is not recorded in the data
    // PRX_DBG_VARS(traj_in.size(), traj_out.size());
    // PRX_DBG_VARS(traj_in.front(), traj_out.front());
    // prx_assert((1 + traj_in.size()) == traj_out.size(), "Trajs not the same size");

    // for (int i = 0; i < std::min(traj_in.size(), max_len); ++i)
    // {
    //   ofs << traj_in[i] << " " << traj_out[i] << "\n";
    // }
    ofs << "\n";
  }
  ofs.close();

  return 0;
}
