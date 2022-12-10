#ifndef MUJOCO_NOT_BUILT
#include "prx/mujoco/mj_simulator.hpp"

namespace prx
{
    mujoco_simulator_t::mujoco_simulator_t(const std::string& model_path) : simulator_t(plant_type::MUJOCO)
    {
        const char* full_model_path = (mj_models_path + model_path).c_str();
        m = mj_loadXML(full_model_path, NULL, NULL, 0);
        if (!m) prx_throw("Error in loading model.")
        PRX_DEBUG_PRINT
        d = mj_makeData(m);

        // Get the simulation step from the model
        simulation_step = m->opt.timestep;

        if (!glfwInit()) prx_throw("Error in initializing GLFW.")

        window = glfwCreateWindow(1200, 900, "MuJoCo", NULL, NULL);
        if (!window) prx_throw("Error in creating GLFW window.")
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
        
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);
        mjv_makeScene(m, &scn, 1000);
        mjr_makeContext(m, &con, mjFONTSCALE_150);

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

    void mujoco_simulator_t::step_simulation(propagate_step step)
    {
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
}
#endif

