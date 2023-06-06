#ifndef TORCH_NOT_BUILT
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    init_random(210896);

    std::string params_file = "examples/mujoco/mushr_trajectory.yaml";
    param_loader params(params_file);
    learned_controller_t controller(params);

    std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>("mushr.xml");
    sim->init_simulator();

    auto context = sim -> get_context("mujoco");
    auto ss = context.first -> get_state_space();
    auto cs = context.first -> get_control_space();
    auto sg = context.first;

    for (double i = 0; i < 1.0/simulation_step; i += 1)
    {
        sim -> step_simulation(propagate_step::FIRST_STEP);
    }

    dirt_t dirt("dirt");
    dirt_specification_t dirt_spec(context.first, context.second);
    dirt_spec.blossom_number = 5;

    dirt_spec.sample_state = [ss](space_point_t& s)
    {
        s->at(0) = uniform_random(-9., 9.);
        s->at(1) = uniform_random(-9., 9.);
        double roll = 0, pitch = 0, yaw = uniform_random(-PRX_PI, PRX_PI);
        Eigen::Quaterniond quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                                * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        s->at(2) = 0.0;
        s->at(3) = quat.w();
        s->at(4) = quat.x();
        s->at(5) = quat.y();
        s->at(6) = quat.z();
    };
    
    dirt_spec.distance_function = [](const space_point_t& a, const space_point_t& b)
    {
        double diff = (a->at(0) - b->at(0)) * (a->at(0) - b->at(0)) + (a->at(1) - b->at(1)) * (a->at(1) - b->at(1));
        return sqrt(diff);
    };

    distance_function_t goal_dist = [](const space_point_t& a, const space_point_t& b)
    {
        double diff = (a->at(0) - b->at(0)) * (a->at(0) - b->at(0)) + (a->at(1) - b->at(1)) * (a->at(1) - b->at(1));
        // Get the Euler angles between the two quaternions
        quaternion_t quat1 = Eigen::Quaterniond(a->at(3), a->at(4), a->at(5), a->at(6));
        quaternion_t quat2 = Eigen::Quaterniond(b->at(3), b->at(4), b->at(5), b->at(6));
        double angular_diff = quat1.angularDistance(quat2);
        diff += angular_diff * angular_diff;
        return sqrt(diff);
    };

    dirt_spec.min_control_steps = 0.5 * (1.0/simulation_step);
    dirt_spec.max_control_steps = 1.0 * (1.0/simulation_step);
    std::cout << dirt_spec.min_control_steps << " " << dirt_spec.max_control_steps << std::endl;

    std::vector<double> start = params["start_state"].as<std::vector<double>>();
    
    double s_roll = start[2], s_pitch = start[3], s_yaw = start[4];
    Eigen::Quaterniond s_quat = Eigen::AngleAxisd(s_roll, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(s_pitch, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(s_yaw, Eigen::Vector3d::UnitZ());

    dirt_query_t dirt_query(ss,cs);
    dirt_query.start_state = ss -> make_point();
    ss -> copy_to_point(dirt_query.start_state);
    dirt_query.start_state ->at(0) =  start[0];
    dirt_query.start_state ->at(1) =  start[1];
    dirt_query.start_state ->at(3) = s_quat.w();
    dirt_query.start_state ->at(4) = s_quat.x();
    dirt_query.start_state ->at(5) = s_quat.y();
    dirt_query.start_state ->at(6) = s_quat.z();


    std::vector<double> goal = params["goal_state"].as<std::vector<double>>();

    double g_roll = goal[2], g_pitch = goal[3], g_yaw = goal[4];
    Eigen::Quaterniond g_quat = Eigen::AngleAxisd(g_roll, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(g_pitch, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(g_yaw, Eigen::Vector3d::UnitZ());
    dirt_query.goal_state = ss -> make_point();
    ss -> copy_to_point(dirt_query.goal_state);
    dirt_query.goal_state -> at(0) = goal[0];
    dirt_query.goal_state -> at(1) = goal[1];
    dirt_query.goal_state -> at(3) = g_quat.w();
    dirt_query.goal_state -> at(4) = g_quat.x();
    dirt_query.goal_state -> at(5) = g_quat.y();
    dirt_query.goal_state -> at(6) = g_quat.z();

    sim -> set_goal(dirt_query.goal_state);
    sim -> set_goal_radius(0.5);

    std::cout << ss -> print_point(dirt_query.start_state, 4) << std::endl;
    std::cout << ss -> print_point(dirt_query.goal_state, 4) << std::endl;

    dirt_query.goal_check = [&](const space_point_t& point)
    {
        return goal_dist(point, dirt_query.goal_state) < 0.5;
    };

    dirt_query.get_visualization = true;

    bool use_random_local_goal = params["use_rlg"].as<bool>();
    space_point_t sample_point = ss -> make_point();
    double duration = params["/learned_controller/control_duration"].as<double>();
    dirt_spec.expand = [&](space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn, bool blossom_expand)
    {
        if (blossom_expand && use_random_local_goal)
        {
            std::vector<std::vector<double>> current_states;
            std::vector<std::vector<double>> local_goals;

            std::vector<double> current_state;
            ss -> copy_vector_from_point(current_state,s);
            std::vector<double> local_goal;

            for (int i = 0; i < bn; i++)
            {
                local_goal.clear();
                dirt_spec.sample_state(sample_point);
                ss -> copy_vector_from_point(local_goal,sample_point);
                current_states.push_back(current_state);
                local_goals.push_back(local_goal);
            }

            auto controls = controller.get_controls(current_states,local_goals);

            trajectory_t traj(ss);
            plan_t plan(cs);

            for (int i = 0; i < bn; i++)
            {
                traj.clear(); plan.clear();
                plan.append_onto_back(duration);
                cs -> copy_point_from_vector(plan.back().control,controls[i]);
                dirt_spec.propagate(s,plan,traj);
                plans.push_back(new plan_t(plan));
                trajs.push_back(new trajectory_t(traj));
            }
        }
        else
        {
            default_expand(s,plans,trajs,bn,sg,dirt_spec.sample_plan,dirt_spec.propagate);
        }
    };

    dirt.link_and_setup_spec(&dirt_spec);
    dirt.preprocess();
    dirt.link_and_setup_query(&dirt_query);

    condition_check_t checker("time", params["duration"].as<double>());
    dirt.resolve_query(&checker);
    dirt.fulfill_query(); 

    std::ofstream fout;
    unsigned counter = 0;
    for (auto& traj : dirt_query.tree_visualization)
    {
        fout.open(output_path + "tree" + std::to_string(counter) + ".txt");
        fout << traj.print(2);
        fout.close();
        counter++;
    }

    fout.open(output_path + "solution_traj.txt");
    fout << dirt_query.solution_traj.print(4);
    fout.close();

    fout.open(output_path + "solution_plan.txt");
    fout << dirt_query.solution_plan.print(4);
    fout.close();

    std::cout << dirt_query.solution_plan.print(4) << std::endl;
}
#else
int main() { return 0;}
#endif