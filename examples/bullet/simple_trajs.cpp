#ifndef BULLET_NOT_BUILT
#include "prx/utilities/defs.hpp"
// #include "prx/bullet_sim/plants/husky.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/bullet_sim/plants/plants.hpp"
#include "prx/bullet_sim/collision_checking/collision_checker.hpp"
#include "prx/bullet_sim/bullet_simulator.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
  try
  {
    simulation_step = 0.01;

    // auto plant = std::dynamic_pointer_cast<bullet_omnirobot_t>(create_system<bullet_omnirobot_t>("racecar"));
    std::string plant_name = "bullet_omnirobot";

    auto system = system_factory_t::create_system(plant_name, plant_name);

    auto plant = std::dynamic_pointer_cast<bullet_omnirobot_t>(system);
    // auto plant = std::dynamic_pointer_cast<racecar_t>(create_system<racecar_t>("racecar"));

    // std::shared_ptr<bullet_simulator_t> bsim;
    //
    std::shared_ptr<bullet_simulator_t> bsim = std::make_shared<bullet_simulator_t>();

    auto sim = bsim;
    // auto sim = bsim.sim;

    bsim->add_urdf(bullet_path + "/data/plane.urdf");

    bsim->add_group({ plant });

    bsim->initialize_simulation();

    // sim -> loadURDF("/Users/Gary/pracsys/bullet3/build_cmake/data/plane.urdf");
    // plant -> setup();
    // plant -> setup(sim);

    // world_model_t<system_group_manager_t, bullet_collision_checker_t> world_model({plant}, {});
    // world_model.create_context("racecar_context",{plant_name},{});
    // world_model.create_context("racecar_context",{"husky"},{});

    auto context = bsim->get_context("bullet_context");

    auto state_space = context.first->get_state_space();

    std::cout << "state_space dim: " << state_space->get_dimension() << std::endl;
    auto sg = context.first;

    auto start_state = context.first->get_state_space()->make_point();

    // start_state->at(0)=0;
    // start_state->at(1)=0;
    // start_state->at(2)=.2;
    context.first->get_state_space()->copy_to_point(start_state);
    std::cout << "Start state: " << start_state << std::endl;

    auto ss = context.system_group->get_state_space();
    auto cs = plant->get_control_space();

    trajectory_t traj(ss);
    auto ctrl = plant->get_control_space()->make_point();
    auto state = ss->make_point();
    // ss -> copy_from_point(start_state);

    ss->copy_point(state, start_state);
    cs->sample(ctrl);
    cs->copy_from_point(ctrl);

    // plant -> propagate(simulation_step, propagate_step::FIRST_STEP);
    plan_t plan(cs);

    plant->update_from_bullet(true);
    plant->compute_control();
    std::vector<double> current_state_vec;

    // bsim.step_simulation(propagate_step::FIRST_STEP);
    // enum propagate_step { FIRST_STEP, MIDDLE_STEP, FINAL_STEP };

    for (int i = 0; i < 1000; ++i)
    {
      sim->step_simulation(simulation_step);
      // bsim -> step_simulation(propagate_step::MIDDLE_STEP);
    }
    // bsim -> step_simulation(propagate_step::FINAL_STEP);

    std::cout << "End propagation!" << std::endl;
    // usleep(6e+7);

    // std::cout << rrt_query.solution_traj.print(4) << std::endl;

    // husky_t* plant_viz = dynamic_cast<husky_t*>(plant.get());
    // racecar_t* plant_viz = dynamic_cast<racecar_t*>(plant.get());
    // bullet_omnirobot_t* plant_viz = dynamic_cast<bullet_omnirobot_t*>(plant.get());
    // plant_viz->visualize_trajectories({traj});
    // plant_viz->visualize_goal(rrt_query.goal_state,rrt_query.goal_region_radius);
    std::cout << "Vis done!" << std::endl;
    // for (int i = 0; i < 1000; i++)
    // rrt_spec.propagate(rrt_query.start_state,rrt_query.solution_plan,rrt_query.solution_traj);
    // usleep(6e+7);
  }
  catch (const prx_assert_t& e)
  {
    std::cout << e.get_message() << std::endl;
  }
  std::cout << "End of program" << std::endl;
  _exit(1);
}
#else
int main(int argc, char* argv[])
{
}
#endif
