#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/math_functions.hpp"
// #include "prx/gtdynamics/utilities/prx_symbols.hpp"
#include "prx/gtdynamics/utilities/symbols_factory.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"


namespace prx
{
	struct goal_factor_params_t
	{
		int T = 0; 
		double theta = 1;
		Eigen::VectorXd goal;
      	Eigen::VectorXd error_scale;

        friend std::ostream& operator<< (std::ostream& os, const goal_factor_params_t& obj) 
        {
        	os << "goal_factor_params_t:" << std::endl;
			os << "\tT:" << obj.T << std::endl;
			os << "\ttheta:" << obj.theta << std::endl;
			os << "\tgoal:" << obj.goal.transpose() << std::endl;
			os << "\terror_scale:" << obj.error_scale.transpose() << std::endl;
			return os;
        }

	};

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
      		system_ptr_t _sys_ptr, 
      		int _ti, 
      		goal_factor_params_t _params
      		)
      		: Base(cost_model, xt_key)
      	{
      		system_ptr = _sys_ptr;
      		auto ss = system_ptr -> get_state_space();
      		
      		prx_assert(_params.T >= _ti, "Total time step T (" << _params.T << ") must be greater than or equal than time step (" << _ti << ").")
      		prx_assert(0 < _params.theta && _params.theta <= 1, "Theta must be \\in (0,1]");
      		prx_assert(_params.error_scale.size() == ss -> get_dimension(), "Error_scale must be the same size as the state space! ");
      		
      		xt_pt = ss -> make_point();
			xt_goal_pt = ss -> make_point();
			error_pt = ss -> make_point();
			ss -> copy_point_from_vector(xt_goal_pt, _params.goal);
			T = _params.T;
			t_i = _ti;
			theta = _params.theta;
			error_scale = _params.error_scale;

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

  			Eigen::VectorXd error_scale;

  			int t_i;
  			int T;
  			double theta;
  	};

}

