#include "prx/mujoco/mj_manipulation.hpp"

namespace prx
{

    void jacobian_steering(mjModel* m, mjData* d, trajectory_t& traj, Eigen::Vector<double, 7> goal_pose, 
                                int body_id, std::vector<int> qpos_inds){
        auto curr_pose = forward_kinematics()
    }

    // Compute Jacobian

    void compute_manipulator_jacobian(mjModel* m, mjData* d, Eigen::Matrix<double, 6, 7>& jac, 
                                        double* jacp, double* jacr, int body_id, std::vector<int> qpos_inds){
        prx_assert(jac.rows() == 6, "Incorrect number of rows in Jacobian.");
        prx_assert(jac.cols() == qpos_inds.size(), "Mismatch between Jacobian columns and inds size: " << jac.cols() << ", " << qpos_inds.size());

        mj_jacBody(m, d, jacp, jacr, body_id);

        int len = 3 * m->nq;
        int curr_row = 0;
        int curr_col = 0;
        int qpos_ind_index = 0;
        for(int i = 0; i < len; i++)
            if (i % m->nq == qpos_inds[qpos_ind_index]){
            jac(curr_row, curr_col) = jacp[i];
            qpos_ind_index = (qpos_ind_index + 1) % qpos_inds.size();
            curr_col += 1;

            if (curr_col == jac.cols()){
                curr_col = 0;
                curr_row += 1;
            }
            }

        for(int i = 0; i < len; i++)
            if (i % m->nq == qpos_inds[qpos_ind_index]){
            jac(curr_row, curr_col) = jacr[i];
            qpos_ind_index = (qpos_ind_index + 1) % qpos_inds.size();
            curr_col += 1;

            if (curr_col == jac.cols()){
                curr_col = 0;
                curr_row += 1;
            }
            }

    }


    std::vector<double> forward_kinematics(mjModel* m, mjData* d, const std::vector<int>& qpos_inds, 
    const std::string& query_link_name, const std::vector<double>& q)
    {
    prx_assert(qpos_inds.size() == q.size(), "Incorrect configuration length provided: " + 
    std::to_string(qpos_inds.size()) + ", " + std::to_string(q.size()) + "\n");
    
    // Save current joint qpos values
    double curr_qpos_vals[qpos_inds.size()] = {};
    for(int i = 0; i < qpos_inds.size(); i++){
        curr_qpos_vals[i] = d->qpos[qpos_inds[i]];
        // std::cout << curr_qpos_vals[i] << std::endl;
    }

    // Set joint qpos values to those specified by q
    for(int i = 0; i < qpos_inds.size(); i++){
        d->qpos[qpos_inds[i]] = q[i];
    }

    // Call MuJoCo's forward kinematics function 
    // This will output the desired pos, quat values in xpos, xquat
    mj_kinematics(m, d);

    auto body_inds = get_body_indices(m, query_link_name);

    // Retrieve tip link position
    std::vector<double> body_pose(body_inds.size());
    for(int i = 0; i < body_inds.size(); i++){
        if (i <= 2){
        body_pose[i] = d->xpos[body_inds[i]];
        }
        else{
        body_pose[i] = d->xquat[body_inds[i]];
        }
    }

    // Reset joint qpos values to the initial values
    for(int i = 0; i < qpos_inds.size(); i++){
        d->qpos[qpos_inds[i]] = curr_qpos_vals[i];
    }

    // Call mj_kinematics to reset variables
    mj_kinematics(m, d); // is this necessary?

    return body_pose;
    }

    std::vector<double> forward_kinematics(mjModel* m, mjData* d, const std::vector<int>& qpos_inds, 
    const std::string& query_link_name, const space_point_t& q)
    {
    // prx_assert(qpos_inds.size() == q->get_dim(), "Incorrect configuration length provided.")=
    
    std::vector<double> q_vec(qpos_inds.size());

    for (int i  = 0; i < q_vec.size(); i++){
        q_vec[i] = q->at(qpos_inds[i]);
    }

    return forward_kinematics(m, d, qpos_inds, query_link_name, q_vec);
    }


}