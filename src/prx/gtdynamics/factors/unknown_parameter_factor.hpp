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

	class unknown_parameter_factor_t : public gtsam::NoiseModelFactor 
	{
		public:

		  // typedefs for value types pulled from keys
		  typedef Eigen::VectorXd Z;
		  typedef Eigen::VectorXd U;

		protected:

		  typedef NoiseModelFactor Base;
		  typedef unknown_parameter_factor_t This;

    		KeyVector z_keys;
    		KeyVector u_keys;


		public:
  			/// @name Constructors
  			/// @{

  		/** Default constructor for I/O only */
  		unknown_parameter_factor_t() {}

  		~unknown_parameter_factor_t() override {}

  		inline Key key() const { return keys_[0]; }

  		/**
  		 *  Constructor
  		 *  @param noiseModel shared pointer to noise model
  		 *  @param key1 by which to look up X value in Values
  		 */
  		unknown_parameter_factor_t(const SharedNoiseModel &noiseModel, Key key1)
  		    : Base(noiseModel, cref_list_of<1>(key1)) {}

		/// @}
		/// @name NoiseModelFactor methods
		/// @{

  		/**
   		 * Calls the 1-key specific version of evaluateError below, which is pure
   		 * virtual so must be implemented in the derived class.
   		*/
  		Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const override 
  		{
    		if(this->active(x)) {
    		  // const X1& x1 = x.at<X1>(keys_[0]);
    		  // const X2& x2 = x.at<X2>(keys_[1]);
    		  	for(auto key : z_keys)
    		  	{
					const Z& zi = x.at<Z>(key);
					
    		  	}

    		  if(H) {
    		    return evaluateError(x1, x2, H);
    		  } else {
    		    return evaluateError(x1, x2);
    		  }
    		} else {
    		  return Vector::Zero(this->dim());
    		}
  		}

  /// @}
  /// @name Virtual methods
  /// @{

  /**
   *  Override this method to finish implementing a unary factor.
   *  If the optional Matrix reference argument is specified, it should compute
   *  both the function evaluation and its derivative in X.
   */
  virtual Vector
  evaluateError(const X &x,
                boost::optional<Matrix &> H = boost::none) const = 0;

  /// @}

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
};// \class NoiseModelFactor1
	

}