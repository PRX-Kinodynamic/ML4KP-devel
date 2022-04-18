#pragma once
#include <array>
#include <numeric>
#include <functional>

#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/math_functions.hpp"
#include "prx/gtdynamics/utilities/symbols_factory.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"

namespace prx
{

	class propagation_factor_t : public gtsam::NoiseModelFactor3<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> 
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
      	propagation_factor_t(const gtsam::noiseModel::Base::shared_ptr &cost_model,
      		gtsam::Key xt0_key, gtsam::Key xt1_key, //gtsam::Key xdt1_key, 
      		gtsam::Key ut1_key,
      		system_ptr_t _sys_ptr)
      		: Base(cost_model, xt0_key, xt1_key, ut1_key)
      	{
      		// ltv = std::dynamic_pointer_cast<ltv_t>(_sys_ptr);
      		ltv = std::dynamic_pointer_cast<plant_t>(_sys_ptr);
      		// ltv = _sys_ptr;
			prx_assert(ltv != nullptr, "Plant is not an ltv_t!");
			xt = ltv -> get_state_space() -> make_point();
			ut = ltv -> get_control_space() -> make_point();
			error_pt = ltv -> get_state_space() -> make_point();
  		}
  		
  		virtual ~propagation_factor_t() {}

  		public:
      	virtual Eigen::VectorXd
  		evaluateError(const X1&, const X2&, const X3&,
      		boost::optional<Eigen::MatrixXd&> H1 = boost::none,
      		boost::optional<Eigen::MatrixXd&> H2 = boost::none,
      		boost::optional<Eigen::MatrixXd&> H3 = boost::none) const override;

		Eigen::VectorXd compute_error(
			Eigen::VectorXd xt0, Eigen::VectorXd xt1, Eigen::VectorXd ut1) const;

			void print(const std::string &s = "",
        	       const gtsam::KeyFormatter &keyFormatter =
        	           gtsam::DefaultKeyFormatter) const override 
    		{
    			std::cout << s << "propagation_factor";
    			Base::print("", keyFormatter);
    		}
		private:
  			using This = propagation_factor_t;
  			using Base = gtsam::NoiseModelFactor3<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>;
			// std::shared_ptr<ltv_t> ltv;
			std::shared_ptr<plant_t> ltv;

			space_point_t xt;
			space_point_t ut;
			space_point_t error_pt;

			Eigen::VectorXd x_plus;
        	Eigen::VectorXd x_minus;
        	Eigen::VectorXd xd_plus;
        	Eigen::VectorXd xd_minus;

        	Eigen::VectorXd u_plus;
        	Eigen::VectorXd u_minus;
        	Eigen::VectorXd ud_plus;
        	Eigen::VectorXd ud_minus;

        	space_point_t mem_aux;
  	};

  	class propagation_factor_4_t : public gtsam::NoiseModelFactor4<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> 
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
      	propagation_factor_4_t(const gtsam::noiseModel::Base::shared_ptr &cost_model,
      		gtsam::Key xt0_key, gtsam::Key xt1_key, //gtsam::Key xdt1_key, 
      		gtsam::Key ut1_key, 
      		gtsam::Key time_key,
      		system_ptr_t _sys_ptr)
      		: Base(cost_model, xt0_key, xt1_key, ut1_key, time_key)
      	{
      		ltv = std::dynamic_pointer_cast<ltv_t>(_sys_ptr);
			prx_assert(ltv != nullptr, "Plant is not an ltv_t!");
			xt = ltv -> get_state_space() -> make_point();
			ut = ltv -> get_control_space() -> make_point();
			error_pt = ltv -> get_state_space() -> make_point();
  		}
  		
  		virtual ~propagation_factor_4_t() {}

  		public:
      	virtual Eigen::VectorXd
  		evaluateError(const X1&, const X2&, const X3&, const X4&,
      		boost::optional<Eigen::MatrixXd&> H1 = boost::none,
      		boost::optional<Eigen::MatrixXd&> H2 = boost::none,
      		boost::optional<Eigen::MatrixXd&> H3 = boost::none,
      		boost::optional<Eigen::MatrixXd&> H4 = boost::none) const override;

		Eigen::VectorXd compute_error(
			Eigen::VectorXd xt0, Eigen::VectorXd xt1, 
			Eigen::VectorXd ut1, Eigen::VectorXd t) const;

		private:
  			using This = propagation_factor_t;
  			using Base = gtsam::NoiseModelFactor4<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>;
			std::shared_ptr<ltv_t> ltv;

			space_point_t xt;
			space_point_t ut;
			space_point_t error_pt;

			Eigen::VectorXd x_plus;
        	Eigen::VectorXd x_minus;
        	Eigen::VectorXd xd_plus;
        	Eigen::VectorXd xd_minus;

        	Eigen::VectorXd u_plus;
        	Eigen::VectorXd u_minus;
        	Eigen::VectorXd ud_plus;
        	Eigen::VectorXd ud_minus;

        	space_point_t mem_aux;
  	};

}