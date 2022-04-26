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
class compute_controls_factor_t : public gtsam::NoiseModelFactor2<Eigen::VectorXd, Eigen::VectorXd> 
{
    private:
    using This = compute_controls_factor_t;
    using Base = gtsam::NoiseModelFactor2<Eigen::VectorXd, Eigen::VectorXd>;
    system_ptr_t system_ptr;

    public:
    /**
     * Construct from joint limits
     * @param q_key joint value key
     * @param cost_model noise model
     * @param lower_limit joint lower limit
     * @param upper_limit joint upper limit
     */
    compute_controls_factor_t(const gtsam::noiseModel::Base::shared_ptr &cost_model,
        gtsam::Key q_key, gtsam::Key u_key,
        system_ptr_t _sys_ptr)
        : Base(cost_model, q_key, u_key)
    {
        system_ptr = _sys_ptr;
    }

  virtual ~compute_controls_factor_t() {}

        public:
            Eigen::VectorXd compute_error(
                Eigen::VectorXd xt0, Eigen::VectorXd ut1) const
            {
                auto ss = system_ptr -> get_state_space();
                auto cs = system_ptr -> get_control_space();
                auto ss_dim = ss -> get_dimension();
                auto cs_dim = cs -> get_dimension();

                ss -> copy_from_vector(xt0);

                auto M = system_ptr -> get_mass_matrix();
                auto C = system_ptr -> get_coriolis_vector();
                auto G = system_ptr -> get_gravity_vector();
                Eigen::VectorXd u = M + C + G;

                return ut1 - u;
                // cs -> copy_from_vector(ut1);
                // system_ptr -> compute_control();

                // cs -> difference(error_pt, xt, error_pt);
                // cs -> copy_vector_from_point(error, error_pt);

            }

  /**
   * Evaluate joint limit errors
   *
   * @param q joint value
   */
    gtsam::Vector evaluateError(
        const Eigen::VectorXd &X1,
        const Eigen::VectorXd &X2,
        boost::optional<gtsam::Matrix&> H1 = boost::none, 
        boost::optional<gtsam::Matrix&> H2 = boost::none) const override 
    {
        auto error = compute_error(X1, X2);
        // std::cout << "prop error: " << error.transpose() << std::endl;
        if (H1)
        {
            std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = 
                std::bind(&compute_controls_factor_t::compute_error, this, 
                            std::placeholders::_1, X2);
            *H1 = math_functions::differentiate(fp, X1);
        }

        if (H2)
        {   
            std::function<Eigen::VectorXd(Eigen::VectorXd)> fp = 
                std::bind(&compute_controls_factor_t::compute_error, this, 
                            X1, std::placeholders::_1);
            *H2 = math_functions::differentiate(fp, X2);
        }
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
      std::cout << s << "compute_controls_factor";
      Base::print("", kf);
    }

};

}  