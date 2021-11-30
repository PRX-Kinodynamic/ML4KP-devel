#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/simulation/plant.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/simulation/plants/types/linear_time_invariant.hpp"

#include <memory>

namespace prx
{
    class ltv_t : public lti_t
    {
        public:

        ltv_t(const ltv_t& other) = default;
        ltv_t(std::string _path);

        virtual ~ltv_t();

        /**
         * @brief      Generate the analitical linearization of the system around xt and ut. This is 
         *             populate matrices A, B.
         *
         * @return     True if successful.
         */
        virtual bool linearize(space_point_t xt, space_point_t ut);

        /**
         * @brief      Compute the derivative without computing y(t).
         *                \dot{x}(t) = Ax(t) + Bu(t)
         *             The derivative (\dot{x}) is saved in the 
         *             state space while the ouput returned.
         *
         * @return     The output \dot{x}(t).
         */
        void linear_derivative();


        /**
         * @brief      Compute the derivative and output.
         *                \dot{x}(t) = Ax(t) + Bu(t)
         *                y(t) = Cx(t) + Du(t)
         *             The derivative (\dot{x}) is saved in the 
         *             state space while the ouput returned.
         *
         * @return     The output y(t).
         */
        Eigen::VectorXd linear_derivative_and_output();

        space_t* get_state_space()
        {
          return ltv_stt_space;
        }

        space_t* get_control_space()
        {
          return ltv_ctr_space;
        }

        Eigen::MatrixXd get_A() const {return A;};
        Eigen::MatrixXd get_B() const {return B;};


        protected:

        virtual void compute_derivative() override
        {
            linear_derivative();
        }
      
        space_t* ltv_stt_space;
        space_t* ltv_ctr_space;

        Eigen::VectorXd x;
        Eigen::VectorXd u;

        Eigen::VectorXd x_plus;
        Eigen::VectorXd x_minus;
        Eigen::VectorXd xd_plus;
        Eigen::VectorXd xd_minus;

        Eigen::VectorXd u_plus;
        Eigen::VectorXd u_minus;
        Eigen::VectorXd ud_plus;
        Eigen::VectorXd ud_minus;

        Eigen::MatrixXd A;
        Eigen::MatrixXd B;

  };

}