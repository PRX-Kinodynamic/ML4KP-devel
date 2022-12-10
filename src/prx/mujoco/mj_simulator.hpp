#pragma once
#ifndef MUJOCO_NOT_BUILT
#include "prx/simulation/simulator.hpp"
#include "prx/mujoco/mj_utils.hpp"
#include "GLFW/glfw3.h"

#include "mujoco/mujoco.h"

namespace prx
{
    class mujoco_simulator_t : public simulator_t
    {
        private:
        mjModel* m;
        mjData* d;
        mjvCamera cam;
        mjvOption opt;
        mjvScene scn;
        mjrContext con;

        GLFWwindow* window;

        public:
        mujoco_simulator_t(const std::string& model_path);

        virtual ~mujoco_simulator_t();

        virtual void step_simulation(propagate_step step) override;

        virtual void reset_simulation() override;
    };
}
#endif