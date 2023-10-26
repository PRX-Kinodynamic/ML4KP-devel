#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/simulation/controllers/learned_controller.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include <iostream>
#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
  auto params = param_loader("examples/roadmaps/mushr.yaml");

  simulation_step = params["simulation_step"].as<double>();
  init_random(params["random_seed"].as<int>());

  std::string plant_name = params["/plant/name"].as<>();
  std::string plant_path = params["/plant/path"].as<>();
  auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
  prx_assert(plant != nullptr, "Plant is nullptr!");

  world_model_t world_model({ plant }, {});
  world_model.create_context("dirt_context", { plant_name }, {});
  auto context = world_model.get_context("dirt_context");

  dirt_specification_t dirt_spec(context.first, context.second);
  auto learned_controller_params = param_loader(params["controller"].as<std::string>());
  learned_controller_t controller(plant, learned_controller_params);
  plant -> set_state_space_bounds(params["/plant/state_space_lower_bound"].as<std::vector<double>>(), params["/plant/state_space_upper_bound"].as<std::vector<double>>());

  dirt_query_t dirt_query(context.first->get_state_space(), context.first->get_control_space());
  dirt_query.goal_state = context.first->get_state_space()->make_point();
  dirt_query.start_state = context.first->get_state_space()->make_point();
  dirt_query.goal_region_radius = params["goal_region_radius"].as<double>();
  dirt_query.goal_check = [&](space_point_t point) {
    double diff2 = (point->at(0) - dirt_query.goal_state->at(0)) * (point->at(0) - dirt_query.goal_state->at(0)) +
                   (point->at(1) - dirt_query.goal_state->at(1)) * (point->at(1) - dirt_query.goal_state->at(1));
    diff2 += norm_angle_pi(point->at(2) - dirt_query.goal_state->at(2)) *
             norm_angle_pi(point->at(2) - dirt_query.goal_state->at(2));
    return std::sqrt(diff2) < dirt_query.goal_region_radius;
  };




    // Create and open a text file
  std::ofstream MyFile("filename.txt");

  const unsigned num_trials = int(1e2);
  unsigned num_successes = 0;
  space_point_t start_state_0 = context.first->get_state_space()->make_point();
  space_point_t goal_state_0 = context.first->get_state_space()->make_point();
  space_point_t final_state_0 = context.first->get_state_space()->make_point();


  //save 0 points
  plant->get_state_space()->copy_point(start_state_0, dirt_query.start_state);
  dirt_spec.sample_state(dirt_query.goal_state);
  plant->get_state_space()->copy_point(goal_state_0, dirt_query.goal_state);
  controller.fulfill_query(dirt_spec, dirt_query);

  std::cout<<"seed start: "<<context.first->get_state_space()->print_point(start_state_0,2)<<std::endl;
  std::cout<<"seed goal:  "<<context.first->get_state_space()->print_point(goal_state_0,2)<<std::endl;
 


  if (dirt_query.solution_traj.size() > 0)
  {
    plant->get_state_space()->copy_point(final_state_0, dirt_query.solution_traj.back());
    MyFile <<"("<<context.first->get_state_space()->print_point(start_state_0,2) << "),(";
    MyFile <<context.first->get_state_space()->print_point(final_state_0,2) <<"),(";
    MyFile <<context.first->get_state_space()->print_point(goal_state_0,2) <<")"<<std::endl;
    std::cout<<"seed image: "<<context.first->get_state_space()->print_point(final_state_0,2)<<std::endl;
    double lipshitz = 0;
    for (int i = 0; i < num_trials; i++)
    {
      dirt_spec.sample_state(dirt_query.start_state);
      while(! default_goal_check(dirt_query.start_state, start_state_0, dirt_query.goal_region_radius)){
        dirt_spec.sample_state(dirt_query.start_state);
      }
        // Write to the file



      double pt_err = space_t::euclidean_2d(dirt_query.start_state, start_state_0, 0, start_state_0->size());

      controller.fulfill_query(dirt_spec, dirt_query);

      if (dirt_query.solution_traj.size() == 0){
        std::cout<<"empty trajectory:";
        std::cout<<"query start: "<<context.first->get_state_space()->print_point(dirt_query.start_state,2)<<std::endl;
        MyFile <<"("<<context.first->get_state_space()->print_point(dirt_query.start_state,2) << ")(NULL)" <<std::endl;

      }else{
        double img_err = space_t::euclidean_2d(dirt_query.solution_traj.back(), final_state_0, 0, final_state_0->size());
        double lip_sample = img_err/pt_err;
        if(lip_sample > lipshitz) lipshitz = lip_sample;

        MyFile <<"("<<context.first->get_state_space()->print_point(dirt_query.start_state,2) << "),(";
        MyFile <<context.first->get_state_space()->print_point(dirt_query.solution_traj.back(),2) <<"),";
        MyFile << lip_sample <<", "<< lipshitz <<std::endl;
      }
      
      output_progress_bar(1.0 * i / num_trials);
    }
    std::cout<<"experimental Lipshitz value: "<< lipshitz <<std::endl;
  }else
  {
    std::cout << "initial trajectory failed" << std::endl;
  }

  // Close the file
  MyFile.close();
  
  // for (int i = 0; i < num_trials; i++)
  // {
  //   dirt_spec.sample_state(dirt_query.goal_state);
  //   controller.fulfill_query(dirt_spec, dirt_query);
  //   if (dirt_query.solution_traj.size() > 0)
  //   {
  //     plant->get_state_space()->copy_point(final_state, dirt_query.solution_traj.back());
  //     if (dirt_query.goal_check(final_state))
  //       num_successes++;
  //   }
    
  //}
  
}