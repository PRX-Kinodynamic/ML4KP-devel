#pragma once

#include <Eigen/Core>
#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/prx_assert.hpp"


namespace prx 
{

/**
** http://www.engr.iupui.edu/~skoskie/ECE684/Riccati_algorithms.pdf
**/

    class care
    {
        public:
        static 
        Eigen::MatrixXd solve(const ref_matrixXd_t& A, const ref_matrixXd_t& B, 
            const ref_matrixXd_t& Q, const ref_matrixXd_t& R, unsigned int max_iterations = 100);
    };

}