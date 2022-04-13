#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/math_functions.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"

namespace prx
{

	class goal_distance_factor_t : public gtsam::NoiseModelFactor1<Eigen::VectorXd> 
	{

  		public:

  		/**
  		 * @brief      Constructs a new instance xt1 = xt0 + xdt1 * dt
  		 *
  		 * @param[in]  gtsam     the cost mode
  		 * @param[in]  xt0_key   The xt0 key
  		 * @param[in]  xt1_key   The xt1 key
  		 * @param[in]  xdt1_key  The xdt1 key
  		 * @param[in]  _sys_ptr  The system pointer
  		 */
      	goal_distance_factor_t(const gtsam::noiseModel::Base::shared_ptr &cost_model,
      		gtsam::Key xt_key,
      		Eigen::VectorXd goal, 
      		system_ptr_t _sys_ptr, 
      		double _t_i)
      		: Base(cost_model, xt_key)
      	{
      		system_ptr = _sys_ptr;
      		auto ss = system_ptr -> get_state_space();
      		xt_pt = ss -> make_point();
			xt_goal_pt = ss -> make_point();
			error_pt = ss -> make_point();
			ss -> copy_point_from_vector(xt_goal_pt, goal);
			t_i = _t_i;
  		}
  		
  		virtual ~goal_distance_factor_t() {}

      	virtual Eigen::VectorXd evaluateError(const X&,
      		boost::optional<Eigen::MatrixXd&> H1 = boost::none) const override;

      	Eigen::VectorXd compute_error(Eigen::VectorXd xt_v) const;
      	void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter =
                   gtsam::DefaultKeyFormatter) const override 
    	{
    	  std::cout << s << "goal_distance_factor";
    	  Base::print("", keyFormatter);
    	}
		private:
  			using This = goal_distance_factor_t;
  			using Base = gtsam::NoiseModelFactor1<Eigen::VectorXd>;

  			system_ptr_t system_ptr;

  			space_point_t xt_pt;
  			space_point_t xt_goal_pt;
  			space_point_t error_pt;

  			double t_i;
  	};

}