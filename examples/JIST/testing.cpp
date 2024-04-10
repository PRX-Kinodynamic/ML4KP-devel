#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/mujoco/mj_utils.hpp"
#include "prx/utilities/heuristics/pose_utils.hpp"
#include "prx/utilities/heuristics/roadmap.hpp"

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

using namespace prx;

int main(int argc, char* argv[])
{
    param_loader params;
    if (argc < 2)
    {
        params = param_loader("examples/JIST/heuristic_test.yaml");
    }
    else
    {
        params = param_loader(argv[1]);
    }
    init_random(params["random_seed"].as<int>());

    std::vector<std::pair<std::string, std::string>> ignored_pairs = params["ignored_pairs"].as<std::vector<std::pair<std::string, std::string>>>();

    std::shared_ptr<prx::mujoco_simulator_t> sim =
        std::make_shared<prx::mujoco_simulator_t>(params["scene_xml_path"].as<std::string>());
    sim->init_simulator();

    sim->add_pair(ignored_pairs);

    auto context = sim->get_context("mujoco");
    auto ss = context.first->get_state_space();
    auto cs = context.first->get_control_space();

    std::shared_ptr<prx::mujoco_simulator_t> ee_sim =
        std::make_shared<prx::mujoco_simulator_t>(params["heuristic_xml_path"].as<std::string>());
    ee_sim->init_simulator();

    auto ee_context = ee_sim->get_context("mujoco");
    auto ee_ss = ee_context.first->get_state_space();
    auto ee_cs = ee_context.first->get_control_space();

    for (int i = 0; i < 1000; i++)
    {
        sim->step_simulation();
        ee_sim->step_simulation();
    }

}