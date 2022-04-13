#pragma once

#include "prx/utilities/general/logger.hpp"

#include "prx/gtdynamics/factors/factors.hpp"
#include "prx/gtdynamics/utilities/prx_symbols.hpp"
// #include "prx/gtdynamics/utilities/utilities_functions.hpp"

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#define GET_FACTOR_NAME_MACRO(FACTOR_CLASS, FACTOR, NAME) \
	if (dynamic_cast<FACTOR_CLASS*>(FACTOR.get())) return NAME;

namespace prx 
{ 
	typedef std::function<bool(const gtsam::Factor* /* Factor */, double /* whitenedError */, size_t /* index */)> factor_filter_t;
		// static const factor_filter_t factor_filter_default = [](const gtsam::Factor *, double, size_t) {return true;};

	inline bool default_factor_filter(const gtsam::Factor *, double, size_t)
	{
		return true;
	}

	std::string get_factor_name(const gtsam::NonlinearFactorGraph::sharedFactor& factor);
	// {
	// 	// if (std::dynamic_pointer_cast<goal_distance_factor_t>(factor))
	// 	GET_FACTOR_NAME_MACRO(goal_distance_factor_t, factor, "goal_factor")
	// 	GET_FACTOR_NAME_MACRO(propagation_factor_t, factor, "propagation_factor")
	// 	GET_FACTOR_NAME_MACRO(space_limit_factor_t, factor, "space_limit_factor")
	// 	GET_FACTOR_NAME_MACRO(gtsam::PriorFactor<Eigen::VectorXd>, factor, "prior_factor")

	// 	return "no_factor_name";
	// }

	class fg_logger_t : public logger_t
	{
		public:
			fg_logger_t(const std::string& file_name, char separator = ' ', 
				const std::string& _nullptr_val = "-")
				: logger_t(file_name, separator = ' ')
			{
				nullptr_val = _nullptr_val;
			}

			virtual ~fg_logger_t()
			{

			}

			void add_graph_errors(const gtsam::NonlinearFactorGraph& graph,
				const gtsam::Values& values, 
				const std::string& extra_values = "",
				const factor_filter_t& print_condition = default_factor_filter,
				const gtsam::KeyFormatter& kf = prx::key_formatter
				);
			// static const factor_filter_t factor_filter_default = default_factor_filter;

			

		private:
			// std::shared_ptr<gtsam::NonlinearFactorGraph> graph_ptr;
			std::string nullptr_val;

	// *
	//  * @brief      Utility function to make a csv file out of a non linear factor graph.
	//  *
	//  * @param[in]  graph           The graph to export to csv
	//  * @param[in]  values          The values to use
	//  * @param[in]  file_name       The file name
	//  * @param[in]  keyFormatter    The key formatter, default is the prx one
	//  * @param[in]  extra_values    The extra values to append at the begin of each row
	//  * @param[in]  printCondition  The print condition: prints a factor if it returns true. Default always returns true. 
	//  * @param[in]  sep             The separator. Default " "
	//  * @param[in]  nullptr_val     The nullptr value: string to print for nullptrs. Default  "-".
	 
	// graph_errors_to_file(const gtsam::NonlinearFactorGraph& graph, 
	// 	const gtsam::Values& values, 
	// 	const std::string& file_name, const gtsam::KeyFormatter& keyFormatter = prx_key_formatter, 
	// 	const std::string& extra_values = "",
	// 	const factor_filter_t& printCondition = factor_filter_default,
	// 	const std::string& sep = " ", const std::string& nullptr_val = "-") const
	// {
	// 	std::ofstream ofs_map;
	// 	ofs_map.open(file_name.c_str(), std::ofstream::trunc);

		

	// 	ofs_map.close();

	// }	

	};  
}