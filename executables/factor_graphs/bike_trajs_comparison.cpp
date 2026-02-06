#include <fstream>
#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/factor_graphs/lie_groups/se2.hpp"
#include "prx/utilities/general/csv_reader.hpp"

using CsvReader = prx::utilities::csv_reader_t;

bool read_trajectory(CsvReader& reader, prx::plan_t& plan, prx::trajectory_t& traj, const std::string plant_name,
                     int length)
{
  using prx::utilities::convert_to;
  //  xi[0], xi[1], ui, dt, Gt, accel

  prx::fg::SE2_t x0;
  Eigen::VectorXd state;
  Eigen::VectorXd control;
  double dt{ 0 };
  int idx{ 0 };
  bool trajs_eof{ true };
  while (reader.has_next_line())
  {
    trajs_eof = false;
    auto line = reader.next_line();
    if (line.size() == 0)
      break;
    if (idx == length)
      break;
    idx++;

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

    dt = convert_to<double>(line[0]);

    traj.push_back(state);
    plan.copy_onto_back(control, dt);
  }
  return trajs_eof;
}

int main(int argc, char* argv[])
{
  prx::param_loader params{};
  params["simulation_step"].set(0.1);
  params["random_seed"].set(112392);
  params["environment"].set("environments/empty.yaml");
  params["filename"].set("${ML4KP_ROS}/data/mujoco/MjMushr_fg_10trajs.txt");
  params.add_opts(argc, argv);

  prx::simulation_step = params["simulation_step"].as<double>();
  prx::init_random(params["random_seed"].as<int>());

  auto obstacles = prx::load_obstacles(params["environment"].as<>());
  std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list = obstacles.second;
  std::vector<std::string> obstacle_names = obstacles.first;

  const std::string plant_name{ "dynamic_vehicle" };
  const std::string plant_path{ "dynamic_vehicle" };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  plant->init(params["plant"]);

  // PRX_DBG_VARS(plant);

  prx::world_model_t world_model({ plant }, { obstacle_list });
  world_model.create_context("context", { plant_name }, { obstacle_names });
  auto context = world_model.get_context("context");
  std::shared_ptr<prx::system_group_t> sg{ prx::system_group(context) };
  std::shared_ptr<prx::collision_group_t> cg{ prx::collision_group(context) };

  prx::space_t* ss{ sg->get_state_space() };
  prx::space_t* cs{ sg->get_control_space() };

  prx::space_point_t start_state{ ss->make_point() };
  prx::plan_t plan{ cs };
  prx::trajectory_t traj_in{ ss };
  prx::trajectory_t traj{ ss };

  CsvReader reader(params["filename"].as<>(), ' ');
  read_trajectory(reader, plan, traj_in, plant_name, 50);

  // plan.from_file(params["plan"].as<>());

  PRX_DBG_VARS(plan);
  ss->copy(start_state, traj_in.front());
  sg->propagate(start_state, plan, traj);

  traj_in.to_file(prx::out_path + "/bike_traj_in.txt");
  traj.to_file(prx::out_path + "/bike_traj_out.txt");
  // PRX_DBG_VARS(traj);

  prx::three_js_group_t* vis_group = new prx::three_js_group_t({ plant }, { obstacle_list });

  std::string body_name = plant_name + "/body";

  // vis_group->add_vis_infos(info_geometry_t::LINE, aorrt_query.tree_visualization, body_name, ss);
  vis_group->add_detailed_vis_infos(prx::info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_animation(traj, ss, start_state);
  vis_group->output_html("dynamic_bike_open_loop.html");

  delete vis_group;

  return 0;
}
