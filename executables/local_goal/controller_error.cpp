#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/planning/planners/dirt.hpp"

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
        dirt_query_t dirt_query(ss,cs);
        dirt_query.start_state = ss -> make_point();
        dirt_query.goal_state  = ss -> make_point();
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

        learned_controller_t controller(params);

        dirt_query.goal_check = [&,dirt_spec,ss](space_point_t s)
        {
            return dirt_spec.distance_function(s,dirt_query.goal_state) < dirt_query.goal_region_radius; 
        };

        std::vector<double> error_x;
        std::vector<double> error_y;
        std::vector<double> error_theta;

        space_point_t current = ss -> make_point();

        for (int i = 0; i < 30; i++)
        {
            double current_error_x = 0;
            double current_error_y = 0;
            double current_error_theta = 0;
            for (int j = 0; j < 100; j++)
            {
                dirt_query.clear_outputs();

                dirt_spec.sample_state(dirt_query.start_state);
                dirt_spec.sample_state(dirt_query.goal_state);

                controller.fulfill_query(dirt_query, dirt_spec);

                ss -> copy_point(current, dirt_query.solution_traj.back());

                current_error_x += std::fabs(dirt_query.goal_state->at(0) - current->at(0));
                current_error_y += std::fabs(dirt_query.goal_state->at(1) - current->at(1));
                current_error_theta += std::fabs(norm_angle_pi(dirt_query.goal_state->at(2) - current->at(2)));
            }

            error_x.push_back(current_error_x/100.0);
            error_y.push_back(current_error_y/100.0);
            error_theta.push_back(current_error_theta/100.0);

            output_progress_bar(1.0*i/30.0);
        }

        // Print mean and standard deviation of the errors
        std::cout << "Mean error in x: " << std::accumulate(error_x.begin(), error_x.end(), 0.0) / error_x.size() << std::endl;
        std::cout << "Mean error in y: " << std::accumulate(error_y.begin(), error_y.end(), 0.0) / error_y.size() << std::endl;
        std::cout << "Mean error in theta: " << std::accumulate(error_theta.begin(), error_theta.end(), 0.0) / error_theta.size() << std::endl;

        // Stddev
        double sq_sum_x = std::inner_product(error_x.begin(), error_x.end(), error_x.begin(), 0.0);`
        double mean_x = std::accumulate(error_x.begin(), error_x.end(), 0.0) / error_x.size();
        double stdev_x = std::sqrt(sq_sum_x / error_x.size() - mean_x * mean_x);

        double sq_sum_y = std::inner_product(error_y.begin(), error_y.end(), error_y.begin(), 0.0);
        double mean_y = std::accumulate(error_y.begin(), error_y.end(), 0.0) / error_y.size();
        double stdev_y = std::sqrt(sq_sum_y / error_y.size() - mean_y * mean_y);

        double sq_sum_theta = std::inner_product(error_theta.begin(), error_theta.end(), error_theta.begin(), 0.0);
        double mean_theta = std::accumulate(error_theta.begin(), error_theta.end(), 0.0) / error_theta.size();
        double stdev_theta = std::sqrt(sq_sum_theta / error_theta.size() - mean_theta * mean_theta);

        std::cout << "Standard deviation in x: " << stdev_x << std::endl;
        std::cout << "Standard deviation in y: " << stdev_y << std::endl;
        std::cout << "Standard deviation in theta: " << stdev_theta << std::endl;
    
    }
    catch(const prx_assert_t& e) 
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }
}
#else
int main() {}
#endif