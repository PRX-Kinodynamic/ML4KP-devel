#ifndef BULLET_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/bullet_sim/plants/plants.hpp"
#include "prx/bullet_sim/collision_checking/collision_checker.hpp"
#include "prx/bullet_sim/bullet_simulator.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/access_roadmap.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    try
    {
        std::string params_file;
        if (argc <= 1)
        {
            prx_throw("This executable needs a parameter file!");
        }
        else 
        {
            params_file = std::string(argv[1]);
        }
        param_loader params(params_file);

        simulation_step = params["simulation_step"].as<double>();
        int random_seed = params["random_seed"].as<int>();
        init_random(random_seed);

        auto system = system_factory_t::create_system("segway","segway");
        auto plant = std::dynamic_pointer_cast<bullet_plant_t>(system);
        prx_assert(plant != nullptr, "Plant is not a bullet_plant!");

        std::vector<double> sv = params["/plant/start_state"].as<std::vector<double>>();
        std::vector<double> gv = params["/plant/goal_state"].as<std::vector<double>>();
        space_point_t start_state = plant->get_state_space()->make_point();
        plant->get_state_space()->copy_point_from_vector(start_state,sv);
        plant->get_state_space()->copy_from_point(start_state);
       
        std::shared_ptr<bullet_simulator_t> sim = std::make_shared<bullet_simulator_t>();
        sim -> add_urdf(bullet_path + "/data/plane.urdf");
        std::string obstacles_file = params["environment"].as<std::string>();
        auto retval = load_obstacles(obstacles_file,sim);
        sim -> add_group({plant});
        sim -> initialize_simulation();
        std::cout << "Start state of the simulator: " << std::endl;
        std::cout << plant->get_state_space()->print_memory(4)  << std::endl;

        auto context = sim -> get_context("bullet_context");
		
		auto ss = context.first->get_state_space();
    	auto cs = context.first -> get_control_space();
        auto sg = context.first;
        auto cg = context.second;

        bullet_collision_group_t bcg(sim);
		auto bcg_ptr = std::make_shared<bullet_collision_group_t>(bcg);
		sim->set_collision_group(bcg_ptr);

        dirt_specification_t dirt_spec(context.first,context.second);
        dirt_query_t dirt_query(ss,cs);
        dirt_query.start_state = ss -> make_point();
        dirt_query.goal_state  = ss -> make_point();
        ss -> copy_to_point(dirt_query.start_state);
        ss -> copy_point_from_vector(dirt_query.goal_state,gv);
        dirt_query.get_visualization = true;

        dirt_query.goal_region_radius = params["goal_radius"].as<double>();

        dirt_spec.distance_function = [&](space_point_t a, space_point_t b)
        {
            std::vector <double> diff = {a->at(0)-b->at(0),a->at(1)-b->at(1),
            norm_angle_pi(a->at(2)-b->at(2))};

            double accum = 0.;
            for (auto v: diff) {
                accum += v*v;
            }
            return sqrt(accum);
        };

        dirt_query.goal_check = [&,dirt_spec,ss](space_point_t s)
        {
            return dirt_spec.distance_function(s,dirt_query.goal_state) < dirt_query.goal_region_radius; 
        };

        dirt_t dirt("dirt");
        dirt_spec.min_control_steps = params["/plant/min_steps"].as<int>();
        dirt_spec.max_control_steps = params["/plant/max_steps"].as<int>();
        dirt_spec.blossom_number = 1;
        dirt_spec.use_pruning = false;

        learned_controller_t controller(params);
        
        
        access_roadmap_t access_roadmap(params);
        // std::string roadmap_dir = "/Users/aravind/Downloads/roadmap_segway_warehouse/";
        // bool success = access_roadmap.build_roadmap_from_file(roadmap_dir, dirt_spec);
        // if (!success) 
        // {
        //     prx_throw("Could not build roadmap from file!");
        // }
        // auto path = access_roadmap.get_shortest_path(17,18);
        // std::cout << "Path: " << std::endl;
        // for (auto v: path)
        // {
        //     std::cout << v << " ";
        // }
        // std::cout << std::endl;

        /*
        space_point_t lg = ss -> make_point();
        dirt_spec.expand = [&](space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn, bool blossom_expand)
        {
            if (blossom_expand)
            {
                std::vector<std::vector<double>> current_states;
                std::vector<std::vector<double>> local_goals;

                std::vector<double> current_state;
                ss -> copy_vector_from_point(current_state,s);
                std::vector<double> local_goal;

                // auto nn = access_roadmap.get_best_node(s,dirt_spec);
                int nn = -1;
                if (nn == -1)
                {
                    local_goal.clear();
                    ss -> sample(lg);
                    ss -> copy_vector_from_point(local_goal,lg);
                    current_states.push_back(current_state);
                    local_goals.push_back(local_goal);
                }
                else
                {
                    ss -> copy_point(lg,access_roadmap.get_point(nn));
                    ss -> copy_vector_from_point(local_goal,lg);
                    current_states.push_back(current_state);
                    local_goals.push_back(local_goal);
                }

                auto controls = controller.get_controls(current_states,local_goals);

                trajectory_t traj(ss);
                plan_t plan(cs);

                for (int i = 0; i < bn; i++)
                {
                    traj.clear(); plan.clear();
                    plan.append_onto_back(controller.get_control_duration());
                    cs -> copy_point_from_vector(plan.back().control,controls[i]);
                    dirt_spec.propagate(s,plan,traj);
                    plans.push_back(new plan_t(plan));
                    trajs.push_back(new trajectory_t(traj));
                }
            }
            else
            {
                default_expand(s,plans,trajs,bn,sg,dirt_spec.sample_plan,dirt_spec.propagate);
            }
        };
        */

        
        condition_check_t checker("time",60);

        dirt.link_and_setup_spec(&dirt_spec);
        dirt.preprocess();
        dirt.link_and_setup_query(&dirt_query);
        dirt.resolve_query(&checker);
        dirt.fulfill_query();

        std::vector<trajectory_t> trajs;
        trajs.push_back(dirt_query.solution_traj);
        sim -> visualize_trajectories(trajs);

        // if (dirt_query.solution_traj.size() > 0)
        // {
        //     const unsigned playback_loops = 100;

        //     for (int i = 0; i < playback_loops; i++)
        //     {
        //         sim -> execute_traj(plant,dirt_query.solution_traj);
        //     }
        // }

       

        // Sleep for a really long time
        sleep(1000000);
    }
    catch(const prx_assert_t& e)
	{
		std::cout<<e.get_message()<<std::endl;
	}
    
}
#else 
int main(int argc, char** argv){}
#endif