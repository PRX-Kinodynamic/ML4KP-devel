#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    init_random(210896);

    // std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>("diffdrive.xml");
    std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>("mushr.xml");
    sim->init_simulator();

    auto context = sim -> get_context("mujoco");
    auto ss = context.first -> get_state_space();
    auto cs = context.first -> get_control_space();

    for (double i = 0; i < 1.0/simulation_step; i += 1)
    {
        sim -> step_simulation(propagate_step::FIRST_STEP);
    }

    dirt_t dirt("dirt");
    dirt_specification_t dirt_spec(context.first, context.second);
    dirt_spec.blossom_number = 5;

    dirt_spec.distance_function = [](const space_point_t& a, const space_point_t& b)
    {
        double diff = (a->at(0) - b->at(0)) * (a->at(0) - b->at(0)) + (a->at(1) - b->at(1)) * (a->at(1) - b->at(1));
        // Get the Euler angles between the two quaternions
        quaternion_t quat1 = Eigen::Quaterniond(a->at(3), a->at(4), a->at(5), a->at(6));
        quaternion_t quat2 = Eigen::Quaterniond(b->at(3), b->at(4), b->at(5), b->at(6));
        auto euler1 = quat1.toRotationMatrix().eulerAngles(0, 1, 2);
        auto euler2 = quat2.toRotationMatrix().eulerAngles(0, 1, 2);
        double euler1_z = euler1(2);
        double euler2_z = euler2(2);
        diff += norm_angle_pi(euler1_z - euler2_z) * norm_angle_pi(euler1_z - euler2_z);
        double dist = quat1.angularDistance(quat2);
        return sqrt(diff);
    };

    dirt_spec.h = [&](const space_point_t& s, const space_point_t& s2)
    {
        return space_t::euclidean_2d(s, s2);
    };

    dirt_spec.min_control_steps = 0.5 * (1.0/simulation_step);
    dirt_spec.max_control_steps = 2.0 * (1.0/simulation_step);
    std::cout << dirt_spec.min_control_steps << " " << dirt_spec.max_control_steps << std::endl;

    double roll = 0, pitch = 0, yaw = 0;
    Eigen::Quaterniond quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    dirt_query_t dirt_query(ss,cs);
    dirt_query.start_state = ss -> make_point();
    dirt_query.goal_state = ss -> make_point();
    ss -> copy_to_point(dirt_query.start_state);
    dirt_query.start_state ->at(0) =  -9.0;
    dirt_query.start_state ->at(1) =  -5.0;
    dirt_query.start_state ->at(3) = quat.w();
    dirt_query.start_state ->at(4) = quat.x();
    dirt_query.start_state ->at(5) = quat.y();
    dirt_query.start_state ->at(6) = quat.z();
    ss -> copy_to_point(dirt_query.goal_state);
    dirt_query.goal_state -> at(0) = 9.0;
    dirt_query.goal_state -> at(1) = 5.0;
    dirt_query.goal_state -> at(3) = quat.w();
    dirt_query.goal_state -> at(4) = quat.x();
    dirt_query.goal_state -> at(5) = quat.y();
    dirt_query.goal_state -> at(6) = quat.z();

    sim -> set_goal(dirt_query.goal_state);
    sim -> set_goal_radius(0.5);

    std::cout << ss -> print_point(dirt_query.start_state, 4) << std::endl;
    std::cout << ss -> print_point(dirt_query.goal_state, 4) << std::endl;

    dirt_query.goal_check = [&](const space_point_t& point)
    {
        return dirt_spec.distance_function(point, dirt_query.goal_state) < 0.5;
    };

    dirt_query.get_visualization = true;

    dirt.link_and_setup_spec(&dirt_spec);
    dirt.preprocess();
    dirt.link_and_setup_query(&dirt_query);

    condition_check_t checker("time", 60.0);
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

    fout.open(output_path + "solution.txt");
    fout << dirt_query.solution_traj.print(4);
    fout.close();

    std::cout << dirt_query.solution_plan.print(4) << std::endl;
}