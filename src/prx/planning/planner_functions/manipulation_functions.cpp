#include "prx/planning/planner_functions/manipulation_functions.hpp"

namespace prx
{
    std::vector<double> calculate_pregrasp(std::vector<double> grasp, vector_t approach_vector){

        vector_t grasp_pos{grasp[0], grasp[1], grasp[2]};
        quaternion_t grasp_ori{grasp[3], grasp[4], grasp[5], grasp[6]};

        Eigen::Matrix4d grasp_matrix;
        grasp_matrix.setIdentity();

        grasp_matrix.block<3, 3>(0, 0) = grasp_ori.normalized().toRotationMatrix();
            
        grasp_matrix.block<3, 1>(0, 3) = grasp_pos;

        Eigen::Matrix4d approach_matrix;
        approach_matrix.setIdentity();

        approach_matrix.block<3, 1>(0, 3) = approach_vector;

        vector_t pregrasp_pos = (grasp_matrix * approach_matrix).block<3, 1>(0, 3);

        std::vector<double> pregrasp{grasp};

        for(int i = 0; i < pregrasp_pos.size(); i++){
            pregrasp[i] = pregrasp_pos[i];
        }
    }
}