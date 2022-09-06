#pragma once
#include "prx/gtdynamics/factors/factors.hpp"
#include "prx/gtdynamics/utilities/fg_logger.hpp"
#include "prx/gtdynamics/utilities/symbols_factory.hpp"
#include "prx/gtdynamics/utilities/utilities_functions.hpp"

PRX_REGISTER_SYMBOL(goal_symbol, "Xg", 0, 0)
PRX_REGISTER_SYMBOL(start_state_symbol, "X0", 0, 0)
PRX_REGISTER_SYMBOL(state_symbol, "Xi", 0, 0)
PRX_REGISTER_SYMBOL(control_symbol, "Ui", 0, 0)
PRX_REGISTER_SYMBOL(time_symbol, "ti", 0, 0)
PRX_REGISTER_SYMBOL(param_symbol, "TH", 0, 0)
