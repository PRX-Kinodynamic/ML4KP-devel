#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <iostream>
#include <boost/optional.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/space.hpp"

#include "prx/gtdynamics/utilities/prx_symbols.hpp"


namespace prx 
{

/**
 * space_limit_factor_t is a class which enforces limits to states
 */
class space_limit_factor_t : public gtsam::NoiseModelFactor1<Eigen::VectorXd> 
{
    private:
    using This = space_limit_factor_t;
    using Base = gtsam::NoiseModelFactor1<Eigen::VectorXd>;
    space_t* ss;
    double epsilon;

    public:
    /**
     * Construct from joint limits
     * @param q_key joint value key
     * @param cost_model noise model
     * @param lower_limit joint lower limit
     * @param upper_limit joint upper limit
     */
    space_limit_factor_t(gtsam::Key q_key,
        const gtsam::noiseModel::Base::shared_ptr &cost_model,
        space_t* _ss, double _epsilon = 0.001)
        : Base(cost_model, q_key)
    {
        ss = _ss;
        epsilon = _epsilon;
    }

  virtual ~space_limit_factor_t() {}

 public:
  /**
   * Evaluate joint limit errors
   *
   * @param q joint value
   */
    gtsam::Vector evaluateError(
        const Eigen::VectorXd &q,
        boost::optional<gtsam::Matrix&> H_q = boost::none) const override 
    {
        auto ss_dim = ss -> get_dimension();
        Eigen::VectorXd error = Eigen::VectorXd::Zero(ss_dim);
        Eigen::MatrixXd H_qp = Eigen::MatrixXd::Zero(ss_dim, ss_dim);

        // std::cout << "q: " << q.transpose() << std::endl;
        for (int i = 0; i < ss_dim; ++i)
        {
            auto q = ss -> at(i);
            // TODO: incorporate limits in angles... e.i ackermann steering
            if ( ss -> topology_at(i) == space_t::topology_t::ROTATIONAL )
            {
                // if (H_q) *H_q = Eigen::MatrixXd::Identity(ss_dim, ss_dim);
                H_qp(i,i) = 0;
                error[i] = 0;
            }
            else // TODO: Other topologies?
            {
                if ( q < ss -> get_lower_bound(i) - epsilon )
                {
                    // std::cout << "LOWER: " << q << "\tbound: " << ss -> get_lower_bound(i) << std::endl;
                    H_qp(i,i) = -1;
                    // if (H_q) *H_q = -1 * Eigen::MatrixXd::Identity(ss_dim, ss_dim);
                    error[i] = ss -> get_lower_bound(i) - q;
                }
                else if ( q <= ss -> get_upper_bound(i) + epsilon )
                {
                    // std::cout << "IN BOUNDS" << std::endl;
                    H_qp(i,i) = 0;
                    // if (H_q) *H_q = Eigen::MatrixXd::Zero(ss_dim, ss_dim);
                    error[i] = 0;
                }
                else
                {
                    // std::cout << "UPPER" << std::endl;
                    H_qp(i,i) = 1;
                    // if (H_q) *H_q = Eigen::MatrixXd::Identity(ss_dim, ss_dim);
                    error[i] = q - ( ss -> get_upper_bound(i) );
                }
            }   
        }
        if (H_q) *H_q = H_qp;
        // if (error.sum() > 0) std::cout << "error: " << error.transpose() << std::endl;
        return error;
    }

    //// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override 
    {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /// print contents
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &kf =
                   gtsam::DefaultKeyFormatter) const override 
    {
      std::cout << s << "space_limit_factor";
      Base::print("", kf);
    }

    private:
    // /// Serialization function
    // friend class boost::serialization::access;
    // template <class ARCHIVE>
    // void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    //   ar &boost::serialization::make_nvp(
    //       "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
    //   ar &low_;
    //   ar &high_;
    // }
};

}  // namespace gtdynamics
