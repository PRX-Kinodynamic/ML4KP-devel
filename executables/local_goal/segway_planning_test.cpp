#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/bullet_sim/plants/plants.hpp"
#include "prx/bullet_sim/collision_checking/collision_checker.hpp"
#include "prx/bullet_sim/bullet_simulator.hpp"

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
        dirt_query.get_visualization = false;

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

        
        condition_check_t checker("time",60);
        dirt.link_and_setup_spec(&dirt_spec);
        dirt.preprocess();
        dirt.link_and_setup_query(&dirt_query);
        dirt.resolve_query(&checker);
    }
    catch(const prx_assert_t& e)
	{
		std::cout<<e.get_message()<<std::endl;
	}
    
}