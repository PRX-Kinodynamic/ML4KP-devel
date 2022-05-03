#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/math_functions.hpp"
#include "prx/gtdynamics/utilities/symbols_factory.hpp"
#include "prx/gtdynamics/factors/goal_distance_factor.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"


namespace prx
{

	class kinetic_energy_factor_t : public gtsam::NoiseModelFactor1<Eigen::VectorXd> 
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
      	kinetic_energy_factor_t(const gtsam::noiseModel::Base::shared_ptr &cost_model,
      		gtsam::Key xt_key,
      		system_ptr_t _sys_ptr, 
      		Eigen::VectorXd& _goal,
      		int _ti,
      		goal_factor_params_t _params
      		)
      		: Base(cost_model, xt_key)
      	{
      		system_ptr = _sys_ptr;
      		auto ss = system_ptr -> get_state_space();
      		
   //    		xt_pt = ss -> make_point();
			// xt_goal_pt = ss -> make_point();
			// error_pt = ss -> make_point();
			// ss -> copy_point_from_vector(xt_goal_pt, _params.goal);
			t_i = _ti;
			// T = _params.T;
			theta = _params.theta;
			
			ss -> copy_from_vector(_goal);
			energy_goal = _sys_ptr -> kinetic_energy();
			std::cout << "kinetic_energy: " << energy_goal << std::endl;
  		}
  		
  		virtual ~kinetic_energy_factor_t() {}

      	Eigen::VectorXd compute_error(Eigen::VectorXd xt_v) const
      	{
      		Eigen::VectorXd error(1);
      		auto ss = system_ptr -> get_state_space();
      		ss -> copy_from_vector(xt_v);
      		
      		error[0] = energy_goal - system_ptr -> kinetic_energy();
      		// error[0] = energy_goal - xt_v[0];
      		// error[0] *= std::pow(theta, t_i);
      		return error;
      	}

      	virtual Eigen::VectorXd evaluateError(const X& xt_v,
      		boost::optional<Eigen::MatrixXd&> H1 = boost::none) const override
      	{
      		if (H1)
			{
				std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = 
					std::bind(&kinetic_energy_factor_t::compute_error, this, 
								std::placeholders::_1);
   				*H1 = math_functions::differentiate(fp, xt_v);
			}

			return compute_error(xt_v);
      	}

      	void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter =
                   gtsam::DefaultKeyFormatter) const override 
    	{
    		std::cout << s << "kinetic_energy_factor";
    		Base::print("", keyFormatter);
    	}
		private:
  			using This = kinetic_energy_factor_t;
  			using Base = gtsam::NoiseModelFactor1<Eigen::VectorXd>;

  			system_ptr_t system_ptr;

  			// int T;
  			int t_i;
  			double theta;
  			double energy_goal;

  	};

}

