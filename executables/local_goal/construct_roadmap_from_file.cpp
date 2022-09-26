#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/access_roadmap.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/dirt.hpp"

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
        prx::timer_t timer; 
        simulation_step = params["simulation_step"].as<double>();
        int random_seed = params["random_seed"].as<int>();
        init_random(random_seed);

        std::string plant_name = params["/plant/name"].as<std::string>();
        std::string plant_path = params["/plant/path"].as<std::string>();
        auto plant = system_factory_t::create_system(plant_name,plant_path);

        std::vector<double> lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
        std::vector<double> upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
        plant -> set_state_space_bounds(lower_bounds,upper_bounds);

        world_model_t world_model({plant},{});
        world_model.create_context("planning_context",{plant_name},{});
        auto context = world_model.get_context("planning_context");

        auto ss = context.first->get_state_space();
        auto cs = context.first->get_control_space();
        auto sg = context.first;

        dirt_specification_t dirt_spec(context.first,context.second);

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

        access_roadmap_t access_roadmap(params);
        std::string roadmap_dir = "/Users/aravind/Downloads/out";
        access_roadmap.build_roadmap_from_file(roadmap_dir, dirt_spec);

        auto path = access_roadmap.get_shortest_path(59,60);
        std::cout << "Path: " << std::endl;
        for (auto v: path)
        {
            std::cout << v << " ";
        }
        std::cout << std::endl;

        std::vector<double> point = {-12.5, 10.0, -1.57};
        space_point_t pt = ss -> make_point();
        ss -> copy_point_from_vector(pt, point);
        auto nn = access_roadmap.get_best_node(pt,dirt_spec);
        std::cout << "Nearest node: " << nn << std::endl;
    }
    catch(const prx_assert_t& e) 
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }
}