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
        mjvCamera cam;
        mjvOption opt;
        mjvScene scn;
        mjrContext con;

        GLFWwindow* window;

        public:
        mujoco_simulator_t(const std::string& model_path);

        virtual ~mujoco_simulator_t();

        void init_simulator();

        virtual void step_simulation(propagate_step step) override;

        virtual void reset_simulation() override;

        void set_state(const MujocoState& state);

        MujocoState get_state();

        mjModel* m;
        mjData* d;

        std::vector<mjJointInfo*> joint_info;
        std::vector<mjActuatorInfo*> actuator_info;

        std::vector<double*> actuator_internal_state;
    };
}
#endif