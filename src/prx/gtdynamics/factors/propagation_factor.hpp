#pragma once
#include <array>
#include <numeric>
#include <functional>

#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/simulation/system_group.hpp"
#include "prx/simulation/playback/plan.hpp"
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
			: Base(cost_model, xt0_key, xt1_key, ut1_key, time_key),
				plan(_sys_ptr -> get_control_space())
		{
			// prx_assert(ltv != nullptr, "Plant is not an ltv_t!");
			xt = _sys_ptr -> get_state_space() -> make_point();
			ut = _sys_ptr -> get_control_space() -> make_point();
			error_pt = _sys_ptr -> get_state_space() -> make_point();
			sys_ptr = _sys_ptr;
		}

		propagation_factor_4_t(const gtsam::noiseModel::Base::shared_ptr &cost_model,
			gtsam::Key xt0_key, gtsam::Key xt1_key, //gtsam::Key xdt1_key, 
			gtsam::Key ut1_key, 
			gtsam::Key time_key,
			std::shared_ptr<system_group_t> _sg, 
			int _t = 0)
			: Base(cost_model, xt0_key, xt1_key, ut1_key, time_key),
				plan(_sg -> get_control_space())
		{
			// prx_assert(ltv != nullptr, "Plant is not an ltv_t!");
			sg = _sg;
			xt = sg -> get_state_space() -> make_point();
			ut = sg -> get_control_space() -> make_point();
			error_pt = sg -> get_state_space() -> make_point();
			x1_fg_pt = sg -> get_state_space() -> make_point();
			x1_prop_pt = sg -> get_state_space() -> make_point();
			x0_pt = sg -> get_state_space() -> make_point();
			t = _t;
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
			// std::shared_ptr<ltv_t> ltv;

			space_point_t xt;
			space_point_t ut;
			space_point_t error_pt;

			int t;
			// Eigen::VectorXd x_prop;
			Eigen::VectorXd x_minus;
			Eigen::VectorXd xd_plus;
			Eigen::VectorXd xd_minus;

			Eigen::VectorXd u_plus;
			Eigen::VectorXd u_minus;
			Eigen::VectorXd ud_plus;
			Eigen::VectorXd ud_minus;

			space_point_t x0_pt;
			space_point_t mem_aux;
			space_point_t x1_fg_pt;
			space_point_t x1_prop_pt;

			system_ptr_t sys_ptr;
			std::shared_ptr<system_group_t> sg;
			mutable plan_t plan;
	};

	class propagation_witness_factor_t : public gtsam::NoiseModelFactor4<Eigen::VectorXd,Eigen::VectorXd,Eigen::VectorXd,Eigen::VectorXd> 
	{
		public:
		propagation_witness_factor_t(const gtsam::noiseModel::Base::shared_ptr &_cost_model,
			gtsam::Key _xt0_key, 
			gtsam::Key _xt1_key, 
			gtsam::Key _ut0_key, 
			gtsam::Key _time_key,
			Eigen::VectorXd _witness,
			double _radius,
			std::shared_ptr<system_group_t> _sg
			)
			: Base(_cost_model, _xt0_key, _xt1_key, _ut0_key, _time_key),
			  plan(_sg -> get_control_space()),
			  witnesses(_witness), radius(_radius)
		{
			sg = _sg;
			// plan.copy_onto_back(ut0, time[0]);
			x_aux_pt = _sg -> get_state_space() -> make_point();
		}

		virtual ~propagation_witness_factor_t() {}

		virtual Eigen::VectorXd evaluateError(const X1&, const X2&, const X3&, const X4&,
 			boost::optional<Eigen::MatrixXd&> H1 = boost::none,
 			boost::optional<Eigen::MatrixXd&> H2 = boost::none,
 			boost::optional<Eigen::MatrixXd&> H3 = boost::none,
 			boost::optional<Eigen::MatrixXd&> H4 = boost::none) const override;

		Eigen::VectorXd compute_error(Eigen::VectorXd x0, Eigen::VectorXd x1, Eigen::VectorXd u0, Eigen::VectorXd t0) const;


		private:
			using This = propagation_witness_factor_t;
			using Base = gtsam::NoiseModelFactor4<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>;

			double radius;
			std::shared_ptr<system_group_t> sg;
			space_point_t x_aux_pt;
			Eigen::VectorXd witnesses;
			mutable plan_t plan;
	};

	class propagation_factor_1_t : public gtsam::NoiseModelFactor1<Eigen::VectorXd> 
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
		propagation_factor_1_t(const gtsam::noiseModel::Base::shared_ptr &_cost_model,
			gtsam::Key _xt0_key, 
			Eigen::VectorXd _xt1, 
			Eigen::VectorXd _ut0, 
			Eigen::VectorXd _time,
			Eigen::VectorXd _params,
			std::shared_ptr<system_group_t> _sg
			)
			: Base(_cost_model, _xt0_key),
			  plan(_sg -> get_control_space()),
			  xt0(1), xt1(_xt1), ut0(_ut0), time(_time), params(_params),
			  unknown_type(X0)
		{
			sg = _sg;
			plan.copy_onto_back(ut0, time[0]);
			x_aux_pt = _sg -> get_state_space() -> make_point();
		}

		propagation_factor_1_t(const gtsam::noiseModel::Base::shared_ptr &_cost_model,
			Eigen::VectorXd _xt0, 
			gtsam::Key  	_xt1_key, 
			Eigen::VectorXd _ut0, 
			Eigen::VectorXd _time,
			Eigen::VectorXd _params,
			std::shared_ptr<system_group_t> _sg
			)
			: Base(_cost_model, _xt1_key),
			  plan(_sg -> get_control_space()),
			  xt0(_xt0), xt1(1), ut0(_ut0), time(_time), params(_params),
			  unknown_type(X1)
		{
			sg = _sg;
			plan.copy_onto_back(ut0, time[0]);
			x_aux_pt = _sg -> get_state_space() -> make_point();
		}

		propagation_factor_1_t(const gtsam::noiseModel::Base::shared_ptr &_cost_model,
			Eigen::VectorXd _xt0, 
			Eigen::VectorXd _xt1, 
			gtsam::Key  	_ut0_key, 
			Eigen::VectorXd _time,
			Eigen::VectorXd _params,
			std::shared_ptr<system_group_t> _sg
			)
			: Base(_cost_model, _ut0_key),
			  plan(_sg -> get_control_space()),
			  xt0(_xt0), xt1(_xt1), ut0(1), time(_time), params(_params),
			  unknown_type(U0)
		{
			sg = _sg;
			x_aux_pt = _sg -> get_state_space() -> make_point();
		}

		propagation_factor_1_t(const gtsam::noiseModel::Base::shared_ptr &_cost_model,
			Eigen::VectorXd _xt0, 
			Eigen::VectorXd _xt1, 
			Eigen::VectorXd _ut0,
			gtsam::Key  	_time_key, 
			Eigen::VectorXd _params,
			std::shared_ptr<system_group_t> _sg
			)
			: Base(_cost_model, _time_key),
			  plan(_sg -> get_control_space()),
			  xt0(_xt0), xt1(_xt1), ut0(_ut0), time(1), params(_params),
			  unknown_type(TIME)
		{
			sg = _sg;
			x_aux_pt = _sg -> get_state_space() -> make_point();
		}

		propagation_factor_1_t(const gtsam::noiseModel::Base::shared_ptr &_cost_model,
			Eigen::VectorXd _xt0, 
			Eigen::VectorXd _xt1, 
			Eigen::VectorXd _ut0,
			Eigen::VectorXd _time,
			gtsam::Key  	_params_key, 
			std::shared_ptr<system_group_t> _sg
			)
			: Base(_cost_model, _params_key),
			  plan(_sg -> get_control_space()),
			  xt0(_xt0), xt1(_xt1), ut0(_ut0), time(_time), params(1),
			  unknown_type(PARAMS)
		{
			sg = _sg;
			plan.copy_onto_back(ut0, time[0]);
			x_aux_pt = _sg -> get_state_space() -> make_point();
		}

  // 		propagation_factor_4_t(const gtsam::noiseModel::Base::shared_ptr &cost_model,
		// 	gtsam::Key xt0_key, 
		// 	gtsam::Key xt1_key, //gtsam::Key xdt1_key, 
		// 	gtsam::Key ut1_key, 
		// 	gtsam::Key time_key,
		// 	std::shared_ptr<system_group_t> _sg
		// 	)
		// 	: Base(cost_model, xt0_key, xt1_key, ut1_key, time_key),
		// plan(_sg -> get_control_space())
		// {
		// }
		
		virtual ~propagation_factor_1_t() {}

		public:
		virtual Eigen::VectorXd
		evaluateError(const X&,
			boost::optional<Eigen::MatrixXd&> H1 = boost::none) const override;

		Eigen::VectorXd compute_error_x0(Eigen::VectorXd vec) const;
		Eigen::VectorXd compute_error_x1(Eigen::VectorXd vec) const;
		Eigen::VectorXd compute_error_u0(Eigen::VectorXd vec) const;
		Eigen::VectorXd compute_error_ti(Eigen::VectorXd vec) const;
		Eigen::VectorXd compute_error_pa(Eigen::VectorXd vec) const;

		private:

			enum prop_unknown_t { X0, X1, U0, TIME, PARAMS };
			using This = propagation_factor_1_t;
			using Base = gtsam::NoiseModelFactor1<Eigen::VectorXd>;
			// std::shared_ptr<ltv_t> ltv;

			space_point_t x_aux_pt;

			prop_unknown_t unknown_type;

			Eigen::VectorXd xt0;
			Eigen::VectorXd xt1;
			Eigen::VectorXd ut0;
			Eigen::VectorXd time;
			Eigen::VectorXd params;
			std::shared_ptr<system_group_t> sg;
			mutable plan_t plan;
	};
}