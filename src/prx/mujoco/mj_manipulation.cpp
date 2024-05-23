#include "prx/mujoco/mj_manipulation.hpp"
#include <unistd.h>

namespace prx
{
    const double damping = 1e-4;
    const double max_jnt_vel = PRX_PI / 4;
    const float integration_dt = 1;

    const float Kpos = 1;
    const float Kori = 1;

    // Currently hard coded
    const std::vector<int> jnt_ids{0, 1, 2, 3, 4, 5, 6};
    const std::vector<int> ctrl_inds{0, 1, 2, 3, 4, 5, 6};

    void steer_test(std::shared_ptr<prx::mujoco_simulator_t> sim, trajectory_t& traj, pose_t goal_pose, int body_id, std::vector<int>& qpos_inds, const config_t& q_init){
        prx_assert(body_id != -1, "ValueError: body_id is invalid");

        int nq = qpos_inds.size();

        std::cout << "JACOBIAN STEERING" << std::endl;
        
        // double *goal_quat = &goal_pose[3];
        double goal_quat[4]{};
        std::copy(goal_pose.begin()+3, goal_pose.end(), goal_quat);

        double curr_quat[4]{};
        double curr_quat_conj[4]{};

        double error_quat[4]{};
        vector_t dx{};
        double dtheta[3]{};

        Eigen::Vector<double, 6> twist{};
        jacobian_t jac{};
        double jacp[sim->m->nq * 3]{};
        double jacr[sim->m->nq * 3]{};
        
        sim->d->qpos[3] = -1.57;
        sim->d->qpos[5] = 1.57;
        sim->d->qpos[6] = -.7853;

        sim->step_simulation();

        config_t q_curr(nq);
        if (q_init.size() > 0){
            q_curr = q_init;
            // TODO: set sim configuration to provided initial configuration
        }
        else{
            // HARD-CODED
            prx_warn("No initial configuration provided. Using current simulation state.");
            std::copy(sim->d->qpos, sim->d->qpos+nq, q_curr.data());
        }
        
        for (int i = 0; i < sim->m->nbody; i++){
            sim->m->body_gravcomp[i] = 1;
        }
        config_t q_start{q_curr};
        for(int i = 0; i < 100000; i++){
            
            std::cout << "iteration " << i << std::endl;
            std::cout << goal_pose.transpose() << std::endl;
            // Eigen::Vector<double, 7> curr_pose = forward_kinematics(sim->m, sim->d, qpos_inds, query_link_name, q_curr);
            vector_t curr_pos = {sim->d->xpos[3*body_id], sim->d->xpos[3*body_id+1],sim->d->xpos[3*body_id+2]};
            
            std::copy(sim->d->xquat+4*body_id, sim->d->xquat+4*body_id+4, curr_quat);

            dx = goal_pose({0, 1, 2}) - curr_pos({0, 1, 2});
            // dx = {0, 0, 0};
            
            // goal_quat = double[4]{0, 1, 0, 0};
            // curr_quat[0] = curr_pose(3, 6);
            mju_negQuat(curr_quat_conj, curr_quat);

            mju_mulQuat(error_quat, goal_quat, curr_quat_conj);

            mju_quat2Vel(dtheta, error_quat, 1.0);

            twist({0, 1, 2}) = dx * Kpos / integration_dt;
            twist({3, 4, 5}) = vector_t{dtheta} * Kori / integration_dt;
            
            /*
            std::cout << "x_goal: " << goal_pose({0, 1, 2}).transpose() << std::endl;
            std::cout << "x_curr: " << curr_pos({0, 1, 2}).transpose() << std::endl;
            std::cout << "dx: " << dx.transpose() << std::endl;

            std::cout << "h_goal: " << goal_pose({3, 4, 5, 6}).transpose() << std::endl;
            std::cout << "h_curr: " << curr_quat[0] << " " << curr_quat[1] << " " << curr_quat[2] << " " << curr_quat[3] << std::endl;
            std::cout << "dtheta: " << dtheta[0] << " " << dtheta[1] << " " << dtheta[2] << " " << std::endl;
            
            std::cout << "error_quat: " << error_quat[0] << " " << error_quat[1] << " " << error_quat[2] << " " << error_quat[3] << " " << std::endl;
            std::cout << "goal_quat: " << goal_quat[0] << " " << goal_quat[1] << " " << goal_quat[2] << " " << goal_quat[3] << " " << std::endl;
            std::cout << "curr_quat_conj: " << curr_quat_conj[0] << " " << curr_quat_conj[1] << " " << curr_quat_conj[2] << " " << curr_quat_conj[3] << " " << std::endl;
            */

            // continue;
            compute_jacobian(sim->m, sim->d, jac, jacp, jacr, body_id, qpos_inds);

            auto identity = Eigen::MatrixXd::Identity(qpos_inds.size(), qpos_inds.size());

            auto jac_pinv_damped = (jac.transpose() * jac + damping * identity).inverse() * jac.transpose();

            Eigen::VectorXd dq = jac_pinv_damped * twist;
            
            // clamping joint velocities
            double dq_max = dq.cwiseAbs().maxCoeff();
            if (dq_max > max_jnt_vel){
                dq *= max_jnt_vel / dq_max;
            }

            // dq += (identity - (jac.transpose() * jac).inverse() * jac.transpose() * jac) * (q_start - q_curr);
            dq += (identity - jac.completeOrthogonalDecomposition().pseudoInverse() * jac) * (q_start - q_curr);

            double qpos[sim->m->nq]{};
            std::copy(sim->d->qpos, sim->d->qpos+sim->m->nq, qpos);

            mj_integratePos(sim->m, qpos, dq.data(), integration_dt);

            // clipping position controls
            for (int i = 0; i < nq; i++){
                qpos[qpos_inds[i]] = std::max(sim->m->jnt_range[2*jnt_ids[i]], 
                std::min(sim->m->jnt_range[2*jnt_ids[i]+1], qpos[qpos_inds[i]]));

                sim->d->ctrl[ctrl_inds[i]] = qpos[qpos_inds[i]];
            }
            sim->step_simulation();

            std::copy(sim->d->qpos, sim->d->qpos+nq, q_curr.data());

            std::cout << "dq: " << dq.transpose() << std::endl;
            // usleep(100000);
        }
        
        // sim->d->site_xmat
    }

    void jacobian_steering(std::shared_ptr<prx::mujoco_simulator_t> sim, trajectory_t& traj, pose_t goal_pose, int body_id, std::vector<int>& qpos_inds, const config_t& q_init){

        int nq = qpos_inds.size();

        std::cout << "JACOBIAN STEERING" << std::endl;

        std::string query_link_name = mj_id2name(sim->m, mjOBJ_BODY, body_id);

        double goal_quat[4]{goal_pose(3, 6)};

        double curr_quat[4]{};
        double curr_quat_conj[4]{};

        double error_quat[4]{};
        vector_t dx{};
        double dtheta[3]{};

        Eigen::Vector<double, 6> twist{};
        jacobian_t jac{};
        double jacp[sim->m->nq * 3]{};
        double jacr[sim->m->nq * 3]{};

        config_t q_curr(nq);
        if (q_init.size() > 0){
            q_curr = q_init;
            // TODO: set sim configuration to provided initial configuration
        }
        else{
            // HARD-CODED
            prx_warn("No initial configuration provided. Using current simulation state.");
            std::copy(sim->d->qpos, sim->d->qpos+nq, q_curr.data());
        }
        
        config_t q_start{q_curr};
        for(int i = 0; i < 100000; i++){
            // ITERATION STARTS HERE
            
            Eigen::Vector<double, 7> curr_pose = forward_kinematics(sim->m, sim->d, qpos_inds, query_link_name, q_curr);

            // Eigen::Vector<double, 7> curr_pose = {sim->d->xpos[27], sim->d->xpos[28], sim->d->xpos[29], sim->d->xquat[36], sim->d->xquat[37], sim->d->xquat[38], sim->d->xquat[39]};

            dx = goal_pose({0, 1, 2}) - curr_pose({0, 1, 2});

            std::cout << "x_goal: " << goal_pose({0, 1, 2}).transpose() << std::endl;
            std::cout << dx.transpose() << std::endl;

            curr_quat[0] = curr_pose(3, 6);
            mju_negQuat(curr_quat_conj, curr_quat);

            mju_mulQuat(error_quat, goal_quat, curr_quat_conj);

            mju_quat2Vel(dtheta, error_quat, 1.0);

            twist({0, 1, 2}) = dx;
            twist({3, 4, 5}) = vector_t{dtheta};

            compute_jacobian(sim->m, sim->d, jac, jacp, jacr, body_id, qpos_inds);

            auto identity = Eigen::MatrixXd::Identity(qpos_inds.size(), qpos_inds.size());

            // std::cout << "J# dx" << std::endl;

            auto jac_pinv_damped = (jac.transpose() * jac + damping * identity).inverse() * jac.transpose();
            // std::cout << (jac_pinv_damped * twist).transpose() << std::endl;

            Eigen::VectorXd dq = jac_pinv_damped * twist;

            double dq_max = dq.cwiseAbs().maxCoeff();
            if (dq_max > max_jnt_vel){
                dq *= max_jnt_vel / dq_max;
            }

            // std::cout << "q_start: " << q_start.transpose() << std::endl;
            // std::cout << "q_curr: " << q_curr.transpose() << std::endl;
            // std::cout << q_init - q_curr << std::endl;
            // std::cout << identity - (jac.transpose() * jac).inverse() * jac.transpose() * jac * (q_start - q_curr) << std::endl;
            dq += (identity - (jac.transpose() * jac).inverse() * jac.transpose() * jac) * (q_start - q_curr);

            double qpos[sim->m->nq]{};
            std::copy(sim->d->qpos, sim->d->qpos+sim->m->nq, qpos);

            /*
            for(int i = 0; i < m->nq; i++){
                std::cout << i << " " << *(qpos+i) << " ";
                std::cout << d->qpos[i] << std::endl;
            }
            std::cout << std::endl;*/

            mj_integratePos(sim->m, qpos, dq.data(), integration_dt);

            for (int i = 0; i < nq; i++){
                qpos[qpos_inds[i]] = std::max(sim->m->jnt_range[2*jnt_ids[i]], 
                std::min(sim->m->jnt_range[2*jnt_ids[i]+1], qpos[qpos_inds[i]]));

                sim->d->ctrl[ctrl_inds[i]] = qpos[qpos_inds[i]];
            }
            sim->step_simulation();

            std::copy(sim->d->qpos, sim->d->qpos+nq, q_curr.data());

            // std::cout << "q: " << q_curr.transpose() << std::endl;
        }
        /*
        for(int i = 0; i < m->nq; i++){
            std::cout << i << " " << *(qpos+i) << " ";
            std::cout << d->qpos[i] << std::endl;
        }
        std::cout << std::endl;
        std::cout << "n_actuators: " << m->nu << std::endl;*/
        // std::cout << *m->jnt_range << m->jnt_range[1] << std::endl;
        // std::cout << dq.maxCoeff() << std::endl;
        // d->qpos->copy()

    }

    

    // Compute Jacobian

    void compute_jacobian(mjModel* m, mjData* d, Eigen::Matrix<double, 6, 7>& jac, 
                                        double* jacp, double* jacr, int body_id, std::vector<int>& qpos_inds){
        prx_assert(jac.rows() == 6, "Incorrect number of rows in Jacobian.");
        prx_assert(jac.cols() == qpos_inds.size(), "Mismatch between Jacobian columns and inds size: " << jac.cols() << ", " << qpos_inds.size());

        mj_jacBody(m, d, jacp, jacr, body_id);
        // mj_jacSite(m, d, jacp, jacr, 0);

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

    Eigen::VectorXd forward_kinematics(mjModel* m, mjData* d, const std::vector<int>& qpos_inds, 
    const std::string& query_link_name, const Eigen::VectorXd& q)
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
        Eigen::VectorXd body_pose(body_inds.size());
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

    Eigen::VectorXd forward_kinematics(mjModel* m, mjData* d, const std::vector<int>& qpos_inds, 
    const std::string& query_link_name, const space_point_t& q)
    {
        // prx_assert(qpos_inds.size() == q->get_dim(), "Incorrect configuration length provided.")=
        
        Eigen::VectorXd q_vec(qpos_inds.size());

        for (int i  = 0; i < q_vec.size(); i++){
            q_vec[i] = q->at(qpos_inds[i]);
        }

        return forward_kinematics(m, d, qpos_inds, query_link_name, q_vec);
    }


}