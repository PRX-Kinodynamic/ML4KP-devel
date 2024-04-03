#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/condition_check.hpp"
#include "prx/utilities/general/param_loader.hpp"

#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/simulation/plants/mushr.hpp"
#include "prx/utilities/general/csv_reader.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
  simulation_step = 0.01;
  init_random(112392);

  const std::string plant_name{ "mushr" };
  const std::string plant_path{ "mushr" };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  auto context = world_model.get_context("context");

  auto system_group = context.first;
  const auto ss = system_group->get_state_space();
  const auto cs = system_group->get_control_space();

  auto start_state = ss->make_point();

  ss->copy(start_state, Eigen::Vector<double, 5>(0, 0, 0, 0, 0));

  plan_t plan(cs);
  trajectory_t traj(ss);

  prx::constants::separating_value = ',';
  const std::string filename{ prx::out_path + "controls.out" };
  prx::utilities::csv_reader_t reader(filename, prx::constants::separating_value);

  // while (reader.has_next_line())
  // {
  //   const prx::utilities::csv_reader_t::Line<double> line{ reader.next_line<double>() };
  //   if (line.size() > 0)
  //   {
  //     plan.copy_onto_back(line, prx::simulation_step);
  //   }
  // }
  // plan.from_file();
  // plan.copy_onto_back(Eigen::Vector2d(0, 0.5), 5);
  // plan.copy_onto_back(Eigen::Vector2d(0.75, 0.5), 5);
  plan.copy_onto_back(Eigen::Vector2d(-1.0, 1.0), 5);
  plan.copy_onto_back(Eigen::Vector2d(0.0, 0.0), 1);

  system_group->propagate(start_state, plan, traj);

  three_js_group_t* vis_group = new three_js_group_t({ plant }, {});

  std::string body_name = plant_name + "/body";

  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj, body_name, ss);

  vis_group->add_animation(traj, ss, start_state);

  vis_group->output_html("mushr.html");

  delete vis_group;
}
