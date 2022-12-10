#pragma once
#ifndef MUJOCO_NOT_BUILT
#include "prx/simulation/simulator.hpp"
#include "prx/mujoco/mj_utils.hpp"
#include "prx/mujoco/mj_plant.hpp"

#include "GLFW/glfw3.h"

#include "mujoco/mujoco.h"

namespace prx
{
    class mujoco_plant_t;
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

        std::vector<mjJointInfo*> joint_info;

        public:
        mujoco_simulator_t(const std::string& model_path);

        virtual ~mujoco_simulator_t();

        void init_simulator();

        virtual void step_simulation(propagate_step step) override;

        virtual void reset_simulation() override;

        // An iterator over all joint infos
        std::pair<std::vector<mjJointInfo*>::iterator, std::vector<mjJointInfo*>::iterator> get_joint_info()
        {
            return std::make_pair(joint_info.begin(), joint_info.end());
        }
    };
}
#endif