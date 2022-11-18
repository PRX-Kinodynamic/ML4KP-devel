#pragma once
#include "prx/factor_graphs/factors/factors.hpp"
#include "prx/factor_graphs/utilities/fg_logger.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/factor_graphs/utilities/utilities_functions.hpp"

PRX_REGISTER_SYMBOL(goal_symbol, "Xg", 0, 0)
PRX_REGISTER_SYMBOL(start_state_symbol, "X0", 0, 0)
PRX_REGISTER_SYMBOL(state_symbol, "Xi", 0, 0)
PRX_REGISTER_SYMBOL(control_symbol, "Ui", 0, 0)
PRX_REGISTER_SYMBOL(time_symbol, "ti", 0, 0)
PRX_REGISTER_SYMBOL(param_symbol, "TH", 0, 0)
