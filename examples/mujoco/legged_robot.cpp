#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/simulation/controller.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

using namespace boost::filesystem;

using namespace prx;

int main(int argc, char* argv[])
{
    param_loader params;
    params = param_loader("examples/tasks/legged.yaml");
    init_random(params["random_seed"].as<int>());

    // intialize simulator with environment xml file
    bool visualize = params["visualize"].as<bool>();
    std::shared_ptr<prx::mujoco_simulator_t> sim =
        std::make_shared<prx::mujoco_simulator_t>(params["xml_path"].as<std::string>(), visualize);
    sim->init_simulator();

    auto context = sim->get_context("mujoco");
    auto ss = context.first->get_state_space();
    // auto sg = context.first;
    auto cs = context.first->get_control_space();
    int n = ss->get_dimension();

    prx::constants::precision = 128;

    for (int i = 0; i < 200; i++)
    {
        sim->step_simulation();
    }

    return 0;
}