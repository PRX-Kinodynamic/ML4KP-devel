/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  MinTorqueFactor.h
 * @brief Factor to minimize torque.
 * @author Alejandro Escontrela
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <string>

namespace prx 
{

	/// MinTorqueFactor is a unary factor which minimizes torque.
	class min_time_factor_t : public gtsam::NoiseModelFactor1<double> 
	{
	 	private:
	 		using This = min_time_factor_t;
	 		using Base = gtsam::NoiseModelFactor1<double>;

		public:

  			min_time_factor_t(gtsam::Key _time_key,
							  const gtsam::noiseModel::Base::shared_ptr &cost_model)
				  : Base(cost_model, _time_key) {}
  			virtual ~min_time_factor_t() {}

		public:
			Eigen::VectorXd evaluateError(
				const double &t_step,
				boost::optional<gtsam::Matrix &> H = boost::none) const override 
			{
				gtsam::Vector error = (gtsam::Vector(1) << t_step).finished();

				if (H) *H = gtsam::I_1x1;

				return error;
  			}

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
	return boost::static_pointer_cast<gtsam::NonlinearFactor>(
		gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
			 const gtsam::KeyFormatter &keyFormatter =
				 gtsam::DefaultKeyFormatter) const override {
	std::cout << s << "min time factor" << std::endl;
	Base::print("", prx::key_formatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
	ar &boost::serialization::make_nvp(
		"NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics
