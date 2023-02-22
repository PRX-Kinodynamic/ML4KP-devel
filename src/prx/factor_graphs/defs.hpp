#pragma once
#include "prx/factor_graphs/factors/factors.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/utilities/utilities_functions.hpp"

#define GET_FACTOR_NAME_MACRO(FACTOR_CLASS, FACTOR, NAME)                                                              \
  if (dynamic_cast<FACTOR_CLASS*>(FACTOR.get()))                                                                       \
    return NAME;

PRX_REGISTER_SYMBOL(goal_symbol, "Xg", 0, 0)
PRX_REGISTER_SYMBOL(start_state_symbol, "X0", 0, 0)
PRX_REGISTER_SYMBOL(state_symbol, "Xi", 0, 0)
PRX_REGISTER_SYMBOL(control_symbol, "Ui", 0, 0)
PRX_REGISTER_SYMBOL(time_symbol, "ti", 0, 0)
PRX_REGISTER_SYMBOL(param_symbol, "TH", 0, 0)
PRX_REGISTER_SYMBOL(param_symbol_X, "Tx", 0, 0)
PRX_REGISTER_SYMBOL(weight_symbol, "Wi", 0, 0)
PRX_REGISTER_SYMBOL(work_space_symbol, "Om", 0, 0);  // \Omega

PRX_REGISTER_SYMBOL(camera, "Ci", 0, 0)
PRX_REGISTER_SYMBOL(feature, "Fi", 0, 0)
PRX_REGISTER_SYMBOL(rotation, "R", 0, 0)
PRX_REGISTER_SYMBOL(translation, "T", 0, 0)
PRX_REGISTER_SYMBOL(position, "p", 0, 0)
PRX_REGISTER_SYMBOL(distance, "d", 0, 0)
PRX_REGISTER_SYMBOL(scale, "s", 0, 0)

PRX_REGISTER_SYMBOL(aruco_marker_rot, "Ar", 0, 0)
PRX_REGISTER_SYMBOL(aruco_marker_tra, "At", 0, 0)
