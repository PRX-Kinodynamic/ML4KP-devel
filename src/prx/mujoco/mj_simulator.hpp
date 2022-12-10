#pragma once
#ifndef MUJOCO_NOT_BUILT
#include "prx/simulation/simulator.hpp"
#include "prx/mujoco/mj_utils.hpp"

#include "mujoco.h"

namespace prx
{
    class mujoco_simulator_t : public simulator_t
    {
        private:
        mjModel* m;
        mjData* d;

        public:
        mujoco_simulator_t(const std::string& model_path);

        virtual ~mujoco_simulator_t();
    };
}
#endif