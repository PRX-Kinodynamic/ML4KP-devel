#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/planning/planners/dirt.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    try
    {
        init_random(21081996);

        std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>("mushr.xml");
        // std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>("mushr_terrain.xml");
        sim->init_simulator();

        auto context = sim -> get_context("mujoco");
        auto ss = context.first -> get_state_space();
        auto cs = context.first -> get_control_space();

        for (double i = 0; i < 1.0/simulation_step; i += 1)
        {
            sim -> step_simulation(propagate_step::FIRST_STEP);
        }

        dirt_specification_t dirt_spec(context.first, context.second);

        space_point_t current = ss -> make_point();
        ss -> copy_to_point(current);
        std::vector<double> xs = linspace(-9.,9.,18);
        std::vector<double> ys = linspace(-5.,5.,10);
        std::vector<double> ts = {0.0, PRX_PI/2, PRX_PI, 3*PRX_PI/2};
        // std::vector<double> ts = linspace(-PRX_PI, PRX_PI, 2);

        trajectory_t traj(ss); plan_t plan(cs);
        std::vector<space_point_t> verification_points;

        std::vector<double> ps = linspace(-.2,.2,3);
        for (auto x : xs)
        {
            for (auto y : ys)
            {
                for (auto t : ts)
                {
                    current -> at(0) = x;
                    current -> at(1) = y;
                    // current -> at(2) = 0.2;
                    double roll = 0, pitch = 0, yaw = t;

                    Eigen::Quaterniond quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

                    current -> at(3) = quat.w();
                    current -> at(4) = quat.x();
                    current -> at(5) = quat.y();
                    current -> at(6) = quat.z();

                    //sim -> step_simulation(propagate_step::FIRST_STEP);

                    if (dirt_spec.valid_state(current))
                    {
                        verification_points.push_back(ss -> clone_point(current));
                    }
                }
            }
        }

        std::ofstream fout;
        std::string out_file = output_path + "points.txt";
        fout.open(out_file);

        for (auto pt : verification_points)
        {
            fout << ss -> print_point(pt,4) << std::endl;
        }

        fout.close();
    }
    catch(const prx_assert_t& e) 
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }
}
#else
int main() {}
#endif