#include "prx/utilities/defs.hpp"
#include "prx/utilities/heuristics/pose_utils.hpp"

#include <boost/filesystem.hpp>

namespace prx
{
    std::vector<double> calculate_pregrasp(std::vector<double> grasp, vector_t approach_vector={0, 0, -.05});
}
