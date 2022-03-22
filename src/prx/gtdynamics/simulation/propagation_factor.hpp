#include <array>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <numeric>

#include "prx/simulation/plants/types/linear_time_variant.hpp"

namespace prx
{

	class propagation_factor_t : public gtsam::NoiseModelFactor4<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> 
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
      		gtsam::Key xt0_key, gtsam::Key xt1_key, gtsam::Key xdt1_key, 
      		gtsam::Key ut1_key,
      		system_ptr_t _sys_ptr)
      		: Base(cost_model, xt0_key, xt1_key, xdt1_key, ut1_key)
      	{
      		ltv = std::dynamic_pointer_cast<ltv_t>(_sys_ptr);
			prx_assert(ltv != nullptr, "Plant is not an ltv_t!");
			xt = ltv -> get_state_space() -> make_point();
			ut = ltv -> get_control_space() -> make_point();
  		}
  		
  		virtual ~propagation_factor_t() {}

  		public:
      	virtual Eigen::VectorXd
  		evaluateError(const X1&, const X2&, const X3&, const X4&,
      		boost::optional<Eigen::MatrixXd&> H1 = boost::none,
      		boost::optional<Eigen::MatrixXd&> H2 = boost::none,
      		boost::optional<Eigen::MatrixXd&> H3 = boost::none,
      		boost::optional<Eigen::MatrixXd&> H4 = boost::none) const override;

		private:
  			using This = propagation_factor_t;
  			using Base = gtsam::NoiseModelFactor4<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>;
			std::shared_ptr<ltv_t> ltv;

			space_point_t xt;
			space_point_t ut;
  	};

}