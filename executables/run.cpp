#ifndef BULLET_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/controllers/random_walk.hpp"
#include "prx/bullet_sim/plants/plants.hpp"
#include "prx/bullet_sim/collision_checking/collision_checker.hpp"
#include "prx/planning/world_model.hpp"

using namespace prx;

int main(int argc, char** argv)
{
    try
	{
		simulation_step = 0.01;
        init_random(11101993);
		
		std::string plant_name = "bullet_omnirobot";
		auto system = system_factory_t::create_system(plant_name, plant_name);
		auto plant = std::dynamic_pointer_cast<bullet_omnirobot_t>(system);
		
        bullet_simulator_t bsim;
		auto sim = bsim.sim;
    	bsim.add_urdf(bullet_path + "/data/plane.urdf");
		bsim.set_group({plant});
		bsim.initialize_simulation();
		
		world_model_t<system_group_manager_t, bullet_collision_checker_t> world_model({plant}, {});
		world_model.create_context("racecar_context",{plant_name},{});

		auto context = world_model.get_context("racecar_context");
		auto ss = context.first->get_state_space();
    	auto cs = context.first -> get_control_space();
		std::cout << "state_space dim: " << ss -> get_dimension() << std::endl;
		auto sg = context.first;

		auto start_state = ss->make_point();
		auto state = ss->make_point();
        ss->copy_to_point(start_state);
		std::cout << "Start state: " << start_state << std::endl;

        plan_t plan(cs);
        plan.append_onto_back(1.0);
        cs->sample(plan.back().control);

        for (int counter = 0; counter < 10; counter++)
        {
            ss->copy_from_point(start_state);
            std::cout << "Start state: " << ss->print_point(start_state) << std::endl;
            std::cout << "Control: " << cs->print_point(plan.back().control) << std::endl;
            plant->compute_control();

            sg -> propagate(start_state,plan,state);

            std::cout << "[Memory] Iter " << counter << ": " << ss->print_memory() << std::endl; 
            std::cout << "[Result] Iter " << counter << ": " << ss->print_point(state) << std::endl;          
            std::cout << "***********************************************************" << s