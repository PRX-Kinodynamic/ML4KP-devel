#pragma once
#ifndef MUJOCO_NOT_BUILT
#include "prx/utilities/defs.hpp"

#include "mujoco/mujoco.h"

namespace prx 
{
    const std::string mj_models_path = models_path + "mujoco/";

    struct mjJointInfo
    {
        std::string name;
        int type;
        bool limited;
        mjtNum range[2];
        int qposadr;
        int dofadr;
    };

    void get_mj_joint_info(mjModel* m, std::vector<mjJointInfo*>& joint_info);

    std::ostream& operator<<(std::ostream& os, const mjJointInfo& info);

    struct MuJoCoState
    {
        mjtNum time;
        std::vector<mjtNum> qpos;
        std::vector<mjtNum> qvel;
        std::vector<mjtNum> act;
    };
}
#endif