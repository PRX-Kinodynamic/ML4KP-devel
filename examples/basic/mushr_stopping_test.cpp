#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
  simulation_step = 0.01;
  init_random(112392);
  prx::param_loader params("examples/basic/mushr_fg.yaml", argc, argv);

  const std::string plant_name{ "mushr" };
  const std::string plant_path{ "mushr" };
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, {});
  world_model.create_context("context", { plant_name }, {});
  auto context = world_model.get_context("context");

  auto sg = context.first;
  auto ss = sg->get_state_space();
  auto cs = sg->get_control_space();
  auto ps = sg->get_parameter_space();

  auto ss_lb = params["plant"]["state_space"]["lower_bound"].as<std::vector<double>>();
  auto ss_ub = params["plant"]["state_space"]["upper_bound"].as<std::vector<double>>();
  ss->set_bounds(ss_lb, ss_ub);
  auto cs_lb = params["plant"]["control_space"]["lower_bound"].as<std::vector<double>>();
  auto cs_ub = params["plant"]["control_space"]["upper_bound"].as<std::vector<double>>();
  cs->set_bounds(cs_lb, cs_ub);
  auto param_values = params["plant"]["parameter_space"]["values"].as<std::vector<double>>();
  ps->copy_from(param_values);

  auto start_state = ss->make_point();
  ss->copy(start_state, Eigen::Vector<double, 4>(1, 0, 0, 0.5));

  double stopping_time = simulation_step;
  sg->compute_stopping_maneuver(start_state, stopping_time);
  std::cout << "Stopping control: " << cs->print_memory() << std::endl;
  std::cout << "Time to stop: " << stopping_time << std::endl;
}