#include "prx/mujoco/mj_utils.hpp"

namespace prx
{
    void get_mj_joint_info(mjModel* m, std::vector<mjJointInfo*>& joint_info)
    {
        for (int i = 0; i < m->njnt; i++)
        {
            mjJointInfo* info = new mjJointInfo();
            info -> name = std::string(m->names + m->name_jntadr[i]);
            info -> type = m->jnt_type[i];
            info -> limited = (bool)m->jnt_limited[i];
            info -> range[0] = m->jnt_range[i * 2];
            info -> range[1] = m->jnt_range[i * 2 + 1];
            info -> qposadr = m->jnt_qposadr[i];
            info -> dofadr = m->jnt_dofadr[i];
            joint_info.push_back(info);
        }
    }

    std::ostream& operator<<(std::ostream& os, const mjJointInfo& info)
    {
        os << "Name: " << info.name << std::endl;
        os << "Type: " << info.type << std::endl;
        os << "Limited: " << info.limited << std::endl;
        os << "Range: " << info.range[0] << ", " << info.range[1] << std::endl;
        return os;
    }
}

