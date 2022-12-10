#ifndef MUJOCO_NOT_BUILT
#include "prx/mujoco/mj_simulator.hpp"

namespace prx
{
    mujoco_simulator_t::mujoco_simulator_t(const std::string& model_path) : simulator_t(plant_type::MUJOCO)
    {
        const char* full_model_path = (mj_models_path + model_path).c_str();
        m = mj_loadXML(full_model_path, NULL, NULL, 0);
        if (!m) prx_throw("Error in loading model.")
        d = mj_makeData(m);

        // Let's do the GLFW stuff later...
    }

    mujoco_simulator_t::~mujoco_simulator_t()
    {
        mj_deleteData(d);
        mj_deleteModel(m);
        mj_deactivate();

        // Let's do the GLFW stuff later...
    }
}
#endif

