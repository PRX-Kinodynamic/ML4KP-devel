#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"   
#include "prx/utilities/general/csv_reader.hpp"
#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
  simulation_step = 0.01;
  init_random(210896);

  auto params = param_loader("plants/mushr.yaml");

  std::string plant_name = params["name"].as<>();
  std::string plant_path = params["path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, {  });
  world_model.create_context("rrt_context", { plant_name }, {  });
  auto context = world_model.get_context("rrt_context");

  auto sg = context.first;
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();
  auto ps = sg->get_parameter_space();

  auto ss_lb = params["state_space"]["lower_bound"].as<std::vector<double>>();
  auto ss_ub = params["state_space"]["upper_bound"].as<std::vector<double>>();
  ss->set_bounds(ss_lb, ss_ub);
  auto cs_lb = params["control_space"]["lower_bound"].as<std::vector<double>>();
  auto cs_ub = params["control_space"]["upper_bound"].as<std::vector<double>>();
  cs->set_bounds(cs_lb, cs_ub);
  auto param_values = params["parameter_space"]["values"].as<std::vector<double>>();\
  ps->copy_from(param_values);
  PRX_DEBUG_VARS(ps->print_memory(8))

  auto start_vec = params["start_state"].as<std::vector<double>>();

  using prx::constants::separating_value;
  using prx::utilities::csv_reader_t;

  std::string plan_file_path = input_path + "plans/mushr_0.txt";
  csv_reader_t reader(plan_file_path, separating_value);

  using Line = std::vector<std::string>;
  plan_t plan(cs);
  trajectory_t traj(ss);
  space_point_t start = ss -> make_point();
  ss->copy(start, start_vec);


  while (reader.has_next_line())
  {
    Line line { reader.next_line()};
    double duration  = prx::utilities::convert_to<double>(line[2]);
    plan.append_onto_back(duration);
    plan.back().control->at(0) = prx::utilities::convert_to<double>(line[0]);
    plan.back().control->at(1) = prx::utilities::convert_to<double>(line[1]);
    PRX_DEBUG_VARS(plan)
    cs->enforce_bounds(plan.back().control);
  }

  sg->propagate(start, plan, traj);
  std::string traj_file_path = out_path + "mushr_traj_0.txt";
  std::ofstream traj_file(traj_file_path);
  traj_file << traj.print(8) << std::endl;
  traj_file.close();
  // PRX_DEBUG_VARS(traj)

  three_js_group_t* vis_group = new three_js_group_t({ plant }, {  });
  std::string body_name = params["name"].as<>() + "/" + params["vis_body"].as<>();
  vis_group->add_detailed_vis_infos(info_geometry_t::FULL_LINE, traj, body_name, ss);
  vis_group->add_animation(traj, ss, start);
  vis_group->output_html("mushr.html");
  delete vis_group;

  std::cout << "End of program!" << std::endl;
}
