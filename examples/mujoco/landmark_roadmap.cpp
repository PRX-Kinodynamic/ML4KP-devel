#ifndef TORCH_NOT_BUILT
#include "prx/utilities/defs.hpp"
#include "prx/mujoco/mj_simulator.hpp"
#include "prx/utilities/learned_modules/learned_controller.hpp"
#include "prx/utilities/learned_modules/landmark_roadmap.hpp"
#include "prx/planning/planners/dirt.hpp"

#include <fstream>

using namespace prx;

std::vector<std::vector<double>> read_comma_separated_file(const std::string& path, const std::string& delimiter = ",")
{
    std::ifstream file(path);
    std::vector<std::vector<double>> dataset;
    std::string line = "";
    while (std::getline(file, line))
    {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, delimiter[0]))
        {
            row.push_back(std::stod(cell));
        }
        dataset.push_back(row);
    }
    return dataset;
}

int main(int argc, char* argv[])
{
    try
    {
        init_random(210896);

        std::string params_file = "examples/mujoco/mushr_trajectory.yaml";
        param_loader params(params_file);

        std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>("mushr.xml");
        sim->init_simulator();

        auto context = sim -> get_context("mujoco");
        auto ss = context.first -> get_state_space();
        auto cs = context.first -> get_control_space();

        dirt_specification_t dirt_spec(context.first,context.second);
        dirt_query_t dirt_query(ss,cs);
        dirt_query.start_state = ss -> make_point();
        dirt_query.goal_state  = ss -> make_point();
        dirt_query.get_visualization = true;

        dirt_query.goal_region_radius = 0.5;

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
            return sqrt(diff);
        };

        learned_controller_t controller(params);

        dirt_query.goal_check = [&,dirt_spec,ss](space_point_t s)
        {
            return dirt_spec.distance_function(s,dirt_query.goal_state) < dirt_query.goal_region_radius; 
        };

        landmark_roadmap_t rrr;
        
        std::ofstream fout;
        
        std::string points_fname = output_path + "points.txt";
        std::vector<std::vector<double>> dataset = read_comma_separated_file(points_fname);
        space_point_t current = ss -> make_point();
        for (auto row: dataset)
        {
            ss -> copy_point_from_vector(current,row);
            rrr.verification_set.push_back(ss -> clone_point(current));
        }

        int verification_set_size = rrr.verification_set.size();
        std::cout << "Verification set size: " << verification_set_size << std::endl;

        prx::timer_t timer;
        timer.reset();
        rrr.set_stretch_factor(2.0);
        rrr.build_roadmap(dirt_query, dirt_spec, controller);
        double time_taken = timer.measure_reset();
        std::cout << "Time taken to build roadmap: " << time_taken << std::endl;
        std::cout << rrr.is_connected() << std::endl;

        // Output graph to file.
        std::string vertex_fname = output_path + "vertices.txt";
        std::string edge_fname = output_path + "edges.txt";
        std::string time_fname = output_path + "time.txt";

        std::ofstream vertex_file(vertex_fname);
        std::ofstream edge_file(edge_fname);
        std::ofstream time_file(time_fname);

        vertex_file << rrr.print_vertices(ss) << std::endl;
        edge_file << rrr.print_edges() << std::endl;
        time_file << time_taken << std::endl;

        vertex_file.close();
        edge_file.close();
        time_file.close();
        
        auto roadmap_edges = rrr.get_all_edges();
        for (auto e = roadmap_edges.first; e != roadmap_edges.second; ++e)
        {
            auto edge = *e;
            std::string traj_fname = output_path + "traj_" + std::to_string(edge.first) + "_" + std::to_string(edge.second) + ".txt";
            std::ofstream fout;
            fout.open(traj_fname);
            fout << rrr.print_edge_traj(edge.first,edge.second,dirt_query,dirt_spec,controller);
            fout.close();
        }
        
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