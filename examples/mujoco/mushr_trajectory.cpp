#ifndef TORCH_NOT_BUILT
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    init_random(21081996);

    std::string params_file = "examples/mujoco/mushr_trajectory.yaml";
    param_loader params(params_file);
    learned_controller_t controller(params);

    std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>("mushr.xml");
    sim->init_simulator();

    auto context = sim -> get_context("mujoco");
    auto ss = context.first -> get_state_space();
    auto cs = context.first -> get_control_space();

    for (double i = 0; i < 1.0/simulation_step; i += 1)
    {
        sim -> step_simulation(propagate_step::FIRST_STEP);
    }

    dirt_query_t query(ss, cs);
    dirt_specification_t spec(context.first, context.second);

    spec.sample_state = [ss](space_point_t& s)
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
    
    spec.distance_function = [](const space_point_t& a, const space_point_t& b)
    {
        double diff = (a->at(0) - b->at(0)) * (a->at(0) - b->at(0)) + (a->at(1) - b->at(1)) * (a->at(1) - b->at(1));
        // Get the Euler angles between the two quaternions
        quaternion_t quat1 = Eigen::Quaterniond(a->at(3), a->at(4), a->at(5), a->at(6));
        quaternion_t quat2 = Eigen::Quaterniond(b->at(3), b->at(4), b->at(5), b->at(6));
        double angular_diff = quat1.angularDistance(quat2);
        diff += angular_diff * angular_diff;
        return sqrt(diff);
    };

    query.goal_region_radius = 0.5;
    query.goal_check = [&,spec](const space_point_t& point)
    {
        return spec.distance_function(point,query.goal_state) < query.goal_region_radius;
    };

    query.clear_outputs();
    query.start_state = ss -> make_point();
    
    ss -> copy_to_point(query.start_state);
    query.goal_state = ss -> make_point();
    
    while(true)
    {
        spec.sample_state(query.goal_state);
        sim -> set_goal(query.goal_state);
        sim -> set_goal_radius(0.5);

        controller.fulfill_query(query,spec);
    }
}
#else
int main() { return 0;}
#endif