#include "prx/utilities/defs.hpp" 
#include "prx/simulation/system_factory.hpp"    
#include "prx/planning/world_model.hpp"     
#include "prx/planning/planners/dirt.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"   
#include "prx/visualization/three_js_group.hpp"     
#include "prx/planning/planner_statistics.hpp"      
#include "prx/simulation/plants/treaded_vehicle_first_order.hpp"
#include "prx/simulation/plants/treaded_vehicle.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/heuristics/medial_axis.hpp"

#include <torch/torch.h>
#include <torch/script.h>
#include <bits/stdc++.h>
#include <fstream>

#include <boost/program_options.hpp>
namespace po = boost::program_options;
using namespace boost;
using namespace prx;

const double map_resolution = 0.1;
const double max_vel = 0.98;

int main(int argc, char* argv[])
{
    po::options_description desc("Second Order Controller - Full");
    desc.add_options()
            ("help,h", "produce help message")
            ("goal,g", po::value<std::string>()->default_value("upper_right"), "Location of Goal Point")
            ("prune,p", po::value<std::string>()->default_value("0"), "0 for no pruning, 1 will include pruning")
            ("output_dir,o", po::value<std::string>()->default_value("out/"), "Path to Output Directory")
            ("blossom,b", po::value<std::string>()->default_value("1"), "Expansions for Blossom")
            ("type,t", po::value<std::string>()->default_value("0"), "Heuristic To Use")
            ("controller,c", po::value<std::string>()->default_value("0"), "Use Learned controller")
            ("window_size,w", po::value<std::string>()->default_value("20"), "Window")
            ("weight,s", po::value<std::string>()->default_value("0.5"), "Weight")
    ;
    // Read arguments
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }
    try
    {
        init_random(101193);
        torch::manual_seed(123456);
        torch::NoGradGuard no_grad;
        simulation_step = 0.1; 
        torch::Device device(torch::kCPU);
        double weight = std::stod(vm["weight"].as<std::string>());
        auto obstacles = load_obstacles("environments/passage.yaml");
        auto obstacle_list = obstacles.second;
        auto obstacle_names = obstacles.first;

        std::string plant_name = "treaded_vehicle";
        auto plant = system_factory_t::create_system(plant_name, plant_name);
        prx_assert(plant != nullptr, "Plant is nullptr!");
        world_model_t<> world_model({plant},{obstacle_list});
        world_model.create_context("disk_context",{plant_name},{obstacle_names});

        std::vector<double> lower = {-14.0,-14.0,-PRX_PI, -0.7, -0.7};
        std::vector<double> upper = { 14.0, 14.0,PRX_PI, 0.7, 0.7}; 
        plant->set_state_space_bounds(lower,upper);
        std::vector <double> action_upper = {0.2, 0.2};

        auto context = world_model.get_context("disk_context");
        context.first -> get_state_space() -> set_bounds(lower, upper);

        dirt_t dirt("dirt");
        dirt_specification_t dirt_spec(context.first,context.second);   //plant, obstacles

        int min_steps = 5;
        int max_steps = 15;
        dirt_spec.min_control_steps = min_steps;
        dirt_spec.max_control_steps = max_steps;
        dirt_spec.blossom_number = 5;
        dirt_spec.use_pruning = false;

        dirt_query_t dirt_query(context.first->get_state_space(), context.first->get_control_space());
        dirt_query.start_state = context.first->get_state_space()->make_point();
        dirt_query.goal_state  = context.first->get_state_space()->make_point();

        dirt_query.start_state -> at(0) = 0;
        dirt_query.start_state -> at(1) = 0;

        dirt_query.goal_state -> at(0) = 0;
        dirt_query.goal_state -> at(1) = 0;

        dirt_query.goal_region_radius = 0.5;    //epsilon for goal check
        dirt_query.get_visualization = true;    //for visualization

        dirt_query.goal_check = [&](space_point_t s)
        {
            // return false;
            std::vector <double> diff = {dirt_query.goal_state->at(0)-s->at(0),
            dirt_query.goal_state->at(1)-s->at(1),
            norm_angle_pi(dirt_query.goal_state->at(2)-s->at(2))};

            double accum = 0.;
            for (auto v: diff) {
                accum += v*v;
            }
            return sqrt(accum) < dirt_query.goal_region_radius; 
        };

        dirt.link_and_setup_spec(&dirt_spec);
        dirt.preprocess();
        dirt.link_and_setup_query(&dirt_query);

        condition_check_t checker("time", 1); 

        std::ofstream fout;
        int stats_runs = 60;        //number of runs
        std::cerr << "HERE\n";
        std::string out_dir = "/home/kushal/dirtmp/out/BC_passage/";

        std::vector <float> xx{-12, 0, 12, -12, 0, 12, 0};
        std::vector <float> yy{12, 12, 12, -12, -12, -12, 0};
        std::vector <float> theta{0, PRX_PI/2, 0, 0, PRX_PI/2, 0, PRX_PI/2};
        // for(int i=0; i<7; i++){
        //     dirt_query.start_state -> at(0) = xx[i];
        //     dirt_query.start_state -> at(1) = yy[i];
        //     bool valid = dirt_spec.valid_state(dirt_query.start_state);
        //     std::cerr << valid << std::endl;
        // }
        // std::cin.get();
        std::string plan_filename;
        for (int i = 0; i < 7; i++)
        {
            for(int j=0; j<7; j++){
                if(i==j) continue;
                dirt_query.start_state -> at(0) = xx[i];
                dirt_query.start_state -> at(1) = yy[i];
                dirt_query.start_state -> at(2) = theta[i];

                dirt_query.goal_state -> at(0) = xx[j];
                dirt_query.goal_state -> at(1) = yy[j];
                dirt_query.goal_state -> at(2) = theta[j];
                // std::cerr << "HERE\n";
                dirt.link_and_setup_spec(&dirt_spec);
                dirt.preprocess();
                dirt.link_and_setup_query(&dirt_query);

                planner_statistics_t stats;
                stats.link_planner(&dirt);
                stats.link_criterion(&checker);
                stats.repeat_data_gathering(30);

                dirt.fulfill_query();
                // plan_filename = 
                // out_dir+"edges/" + std::to_string(i) +".txt";
                // fout.open(plan_filename);

                // auto iter_bounds = dirt.tree.edges();
                // for(auto iter = iter_bounds.first; iter!=iter_bounds.second; iter++)
                // {
                //     fout << (*iter)->get_source() << " " << (*iter)->get_target() << std::endl;
                //     // break;
                // }
                // fout.close();
                // plan_filename = 
                // out_dir+ "vertices/" + std::to_string(i) +".txt";
                // fout.open(plan_filename);

                // auto vertice_iter = dirt.tree.vertices();
                // for(auto iter = vertice_iter.first; iter!=vertice_iter.second; iter++)
                // {
                //     fout << (*iter)->get_index() << " \n";
                //     auto s = (*iter)->point;
                //     fout << context.first->get_state_space()->print_point(s, 10) << std::endl;
                //     // break;
                // }
                // fout.close();

                plan_filename = 
                out_dir + std::to_string(i*7+j) +".txt";
                fout.open(plan_filename);
                fout<<dirt_query.solution_traj.print(10);

                fout.close();

                dirt_query.clear_outputs();
                dirt.reset();
            }
        } 
    }
    catch(const prx_assert_t& e)
    {
        std::cout << e.get_message() << '\n';
    }
    std::cout << "End of program" << std::endl;
}