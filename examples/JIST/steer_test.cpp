// #ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_utils.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/rrt_star.hpp"

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

    std::shared_ptr<prx::mujoco_simulator_t> sim =
        std::make_shared<prx::mujoco_simulator_t>(params["scene_xml_path"].as<std::string>());
    sim->init_simulator();

    auto context = sim->get_context("mujoco");
    auto ss = context.first->get_state_space();
    auto cs = context.first->get_control_space();
    for (int i = 0; i < 100; i++)
    {
        sim->step_simulation();
    }

    std::vector<std::string> joint_names = params["joint_names"].as<std::vector<std::string>>();
    auto arm_qpos_inds = get_qpos_indices(sim->m, mjOBJ_JOINT, joint_names);

    auto start_state = ss->make_point();
    ss->copy_to(start_state);

    auto start_state_vec = Vec(start_state);
    std::cout << "ARM CONFIG: " << start_state_vec(arm_qpos_inds) << typeid(start_state_vec(arm_qpos_inds)).name() << std::endl;
    std::cout << "SIZE: " << start_state_vec.size() << std::endl;

    vector_t q_init = start_state_vec(arm_qpos_inds);
    // vector_t q_goal = params["goal_config"].as<vector_t>();

    int body = mj_name2id(sim->m, mjOBJ_BODY, "hand");

    double jacp[3 * sim->m->nv] = {0};
    double jacr[3 * sim->m->nv] = {0};
    std::cout << 3 * sim->m->nv << std::endl;
    // double jacr[28] = {0};
    mj_jacBody(sim->m, sim->d, jacp, jacr, body);

    for (int i = 0; i < 3; i++){
      for(int j = 0; j < sim->m->nv; j++){
        std::cout << i * 7 + j << ": " << jacp[i * 7 + j] << "\n";
      }
      std::cout << std::endl;
    }

    for (int i = 0; i < 3; i++){
      for(int j = 0; j < sim->m->nv; j++){
        std::cout << i * 7 + j << ": " << jacr[i * 7 + j] << "\n";
      }
      std::cout << std::endl;
    }

    for(int i = 0; i < sim->m->nv; i++){
      int bodyid = sim->m->dof_bodyid[i];
      if(bodyid != -1){
        std::cout << "DOF " << i << ": " << mj_id2name(sim->m, mjOBJ_BODY, bodyid) << std::endl;
      }
    }

    Eigen::Matrix<double, 6, 7> jac{};
    double temp_jacp[3 * sim->m->nv]{};
    double temp_jacr[3 * sim->m->nv]{};    
    compute_manipulator_jacobian(sim->m, sim->d, jac, temp_jacp, temp_jacr, body, arm_qpos_inds);

    std::cout << jac << std::endl;

    std::cout << "End of program!" << std::endl;
}
