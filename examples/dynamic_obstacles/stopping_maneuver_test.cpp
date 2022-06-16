#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"

using namespace prx;

int main (int argc, char* argv[])
{
    simulation_step = 0.1;
    init_random(0);

    std::string plant_name = "treaded_vehicle";
    std::string plant_path = "treaded_vehicle";
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    std::shared_ptr<world_model_t> sim(new world_model_t({plant},{}));
    sim -> create_context("dirt_context",{plant_name},{});
    auto context = sim -> get_context("dirt_context");
    auto sg = context.first;
    auto ss = sg -> get_state_space();
    auto cs = sg -> get_control_space();

    space_point_t point = ss->make_point();
    double t_H = std::atof(argv[1]);
    double vL, vR;
    std::vector<double> desired_acceleration;
    int num_trials = 1e6;
    int successes = 0;

    trajectory_t traj(ss);
    plan_t plan(cs);

    for (int i = 0; i < num_trials; ++i)
    {
        traj.clear(); plan.clear();
        plan.append_onto_back(t_H);

        ss->sample(point);
        vL = point -> at(3);
        vR = point -> at(4);

        desired_acceleration = {
            (0 - vL)/t_H, (0 - vR)/t_H
        };

        cs->copy_point_from_vector(plan.back().control, desired_acceleration);
        cs->enforce_bounds(plan.back().control);
        sg->propagate(point,plan,traj);
        ss->copy_point(point, traj.back());
        vL = point -> at(3); 
        vR = point -> at(4);

        if (std::sqrt(vL*vL + vR*vR) < PRX_EPSILON) successes++;
        output_progress_bar(i*1.0/num_trials);
    }
    std::cout << "Ratio of successful stops for t_H = " << t_H << ": " << (double)successes/num_trials << std::endl;
}
