#include "prx/gtdynamics/utilities/fg_logger.hpp"


namespace prx 
{ 
	std::string get_factor_name(const gtsam::NonlinearFactorGraph::sharedFactor& factor)
	{
		// if (std::dynamic_pointer_cast<goal_distance_factor_t>(factor))
		GET_FACTOR_NAME_MACRO(goal_distance_factor_t, factor, "goal_factor")
		GET_FACTOR_NAME_MACRO(bang_bang_factor_t, factor, "bang_bang_factor_t")
		GET_FACTOR_NAME_MACRO(propagation_factor_t, factor, "propagation_factor")
		GET_FACTOR_NAME_MACRO(propagation_factor_4_t, factor, "propagation_factor")
		GET_FACTOR_NAME_MACRO(propagation_factor_1_t, factor, "propagation_factor")
		GET_FACTOR_NAME_MACRO(space_limit_factor_t, factor, "space_limit_factor")
		GET_FACTOR_NAME_MACRO(kinetic_energy_factor_t, factor, "kinetic_energy_factor")
		GET_FACTOR_NAME_MACRO(potential_energy_factor_t, factor, "potential_energy_factor")
		GET_FACTOR_NAME_MACRO(state_propagation_factor_t, factor, "state_propagation_factor")
		GET_FACTOR_NAME_MACRO(gtsam::PriorFactor<Eigen::VectorXd>, factor, "prior_factor")

		return "no_factor_name";
	}

	void fg_logger_t::add_graph_errors(const gtsam::NonlinearFactorGraph& graph,
				const gtsam::Values& values, 
				const std::string& extra_values,
				const factor_filter_t& print_condition,
				const gtsam::KeyFormatter& kf
				)
			{
				int i = 0; 
				for (const gtsam::NonlinearFactorGraph::sharedFactor& factor : graph)
				{
					const double errorValue = (factor != nullptr ? factor -> error(values) : .0);
					if (!print_condition(factor.get(), errorValue, i)) continue;

					ofs_logger << extra_values << sep;
					ofs_logger << i << sep;
    				if (factor == nullptr) 
    				{
						ofs_logger << nullptr_val << sep;
						ofs_logger << nullptr_val << sep;
    				}
    				else
    				{
						// ofs_logger << factor->print(ss.str(), keyFormatter) << sep;
						ofs_logger << get_factor_name(factor) << sep;
						ofs_logger << std::fixed << errorValue << sep;
    				}
    				ofs_logger << "\n";
    				i++;
				}
			}
}