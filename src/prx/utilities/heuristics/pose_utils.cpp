#include "prx/utilities/heuristics/pose_utils.hpp"

namespace prx{
    space_t* pose_space()
    {
        double _cart_x = 0;
        double _cart_y = 0;
        double _cart_z = 0;
        double _quat_w = 0;
        double _quat_x = 0;
        double _quat_y = 0;
        double _quat_z = 0;

        std::vector<double*> state_memory = {&_cart_x, &_cart_y, &_cart_z, &_quat_w, &_quat_x, &_quat_y, &_quat_z};

        return new space_t{"EEEQQQQ", state_memory, "pose_space"};
    }
}