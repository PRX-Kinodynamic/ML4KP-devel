#include "prx/utilities/defs.hpp"
#include "prx/utilities/heuristics/pose_utils.hpp"

#include <boost/filesystem.hpp>

namespace prx
{
    typedef Eigen::Matrix<double, 6, 7> jacobian_t;
    typedef Eigen::Vector<double, 7> pose_t;
    typedef Eigen::VectorXd config_t;

    Eigen::VectorXd calculate_pregrasp(pose_t grasp, vector_t approach_vector={0, 0, -.05});

    Eigen::VectorXd compose_transformations(pose_t pose1, pose_t pose2);

    double task_distance(pose_t pose_a, pose_t pose_b, std::vector<pose_t> ee_transforms);
}
