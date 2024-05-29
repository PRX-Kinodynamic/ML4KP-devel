#include "prx/planning/planner_functions/manipulation_functions.hpp"

namespace prx
{
    // Pre-allocated variables

    // compose_transformations
    vector_t x_1;
    vector_t x_2;
    quaternion_t ori_1;
    quaternion_t ori_2;
    Eigen::Matrix4d pose1_mat;
    Eigen::Matrix4d pose2_mat;

    // task_distance
    pose_t ee_a;
    pose_t ee_b;

    Eigen::VectorXd calculate_pregrasp(pose_t grasp, vector_t approach_vector){

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

        Eigen::VectorXd pregrasp{grasp};

        for(int i = 0; i < pregrasp_pos.size(); i++){
            pregrasp[i] = pregrasp_pos[i];
        }

        return pregrasp;
    }

    // Poses are expected as a 7-tuple which contains the cartesian coordinates
    // and quaternions, concatenated
    Eigen::VectorXd compose_transformations(pose_t pose1, pose_t pose2){
        x_1 = vector_t{pose1({0, 1, 2})};
        ori_1 = quaternion_t{pose1({3, 4, 5, 6})};

        pose1_mat.setIdentity();

        pose1_mat.block<3, 3>(0, 0) = ori_1.normalized().toRotationMatrix();
            
        pose1_mat.block<3, 1>(0, 3) = x_1;

        x_2 = vector_t{pose2({0, 1, 2})};
        ori_2 = quaternion_t{pose2({3, 4, 5, 6})};

        pose2_mat.setIdentity();

        pose2_mat.block<3, 3>(0, 0) = ori_2.normalized().toRotationMatrix();
            
        pose2_mat.block<3, 1>(0, 3) = x_2;

        vector_t x = (pose1_mat * pose2_mat).block<3, 1>(0, 3);
        quaternion_t q{(pose1_mat * pose2_mat).block<3, 3>(0, 0)};

        Eigen::VectorXd pose(7);
        pose << x, q.coeffs()({3, 0, 1, 2});

        return pose;
    }

    double task_distance(pose_t pose_a, pose_t pose_b, std::vector<pose_t> ee_transforms){
        double dist = 0;

        for (auto ee_transform : ee_transforms){
            ee_a = compose_transformations(pose_a, ee_transform);
            ee_b = compose_transformations(pose_b, ee_transform);

            double euclidean = 0;
            for (int i = 0; i < 3; i++){
                euclidean += std::pow(ee_a[i] - ee_b[i], 2.0);
            }
            euclidean = std::sqrt(euclidean);

            dist += euclidean;
        }

        return dist/ee_transforms.size();
        
    }
}