#ifndef MUJOCO_NOT_BUILT
#include "prx/mujoco/mj_simulator.hpp"

namespace prx
{
    mujoco_simulator_t::mujoco_simulator_t(const std::string& model_path) : simulator_t(plant_type::MUJOCO)
    {
        std::string full_model_path = mj_models_path + model_path;
        m = mj_loadXML(full_model_path.c_str(), NULL, NULL, 0);
        if (!m) prx_throw("Error in loading model.")
        PRX_DEBUG_PRINT
        d = mj_makeData(m);

        // Get the simulation step from the model
        simulation_step = m->opt.timestep;

        if (!glfwInit()) prx_throw("Error in initializing GLFW.")

        window = glfwCreateWindow(800, 600, "MuJoCo", NULL, NULL);
        if (!window) prx_throw("Error in creating GLFW window.")
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
        
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);
        mjv_makeScene(m, &scn, 1000);
        mjr_makeContext(m, &con, mjFONTSCALE_150);

        if (m -> na != 0)
        {
            prx_throw("This system has actuator states, which is not supported right now.")
        }

        get_mj_joint_info(m, joint_info);
        for (auto& info : joint_info)
        {
            std::cout << *info << std::endl;
        }

        get_mj_actuator_info(m, actuator_info);
        for (auto& info : actuator_info)
        {
            std::cout << *info << std::endl;
        }
    }

    mujoco_simulator_t::~mujoco_simulator_t()
    {
        mjv_freeScene(&scn);
        mjr_freeContext(&con);
        glfwTerminate();

        mj_deleteData(d);
        mj_deleteModel(m);
        mj_deactivate();
    }

    void mujoco_simulator_t::init_simulator()
    {
        system_groups -> link_simulator(this);
        std::string context_name = "mujoco";
        std::vector<system_ptr_t> context_systems;

        system_ptr_t system;
        system.reset(new mujoco_plant_t("mujoco_plant"));
        context_systems.push_back(system);
        auto mj_ptr = std::dynamic_pointer_cast<mujoco_plant_t>(system);
        auto sim_ptr = std::static_pointer_cast<mujoco_simulator_t>(this -> shared_ptr());
        mj_ptr -> initialize(sim_ptr);

        system_groups -> add_system_group(context_name, context_systems);

        // TODO: Need to set up collision stuff here.
        collision_groups.reset(new collision_checker_t());
        collision_groups -> add_collision_group(context_name, context_systems, {});
    }

    void mujoco_simulator_t::step_simulation(propagate_step step)
    {
        // Set the warmstart acceleration to be zero (for determinism)
        for (int i = 0; i < m->nv; i++)
        {
            d -> qacc_warmstart[i] = 0;
        }
        mj_step(m, d);
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    void mujoco_simulator_t::reset_simulation()
    {
    }

    MujocoState mujoco_simulator_t::get_state()
    {
        MujocoState s;
        s.time = d->time;
        for (int i = 0; i < m->nq; i++)
        {
            s.qpos.push_back(d->qpos[i]);
        }
        for (int i = 0; i < m->nv; i++)
        {
            s.qvel.push_back(d->qvel[i]);
        }
        for (int i = 0; i < m->na; i++)
        {
            s.act.push_back(d->act[i]);
        }
        for (int i = 0; i < m->nu; i++)
        {
            s.ctrl.push_back(d->ctrl[i]);
        }
        return s;
    }
}
#endif

