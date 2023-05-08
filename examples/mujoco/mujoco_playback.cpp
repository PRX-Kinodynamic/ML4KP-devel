#include "prx/mujoco/mj_simulator.hpp"
#include "prx/simulation/playback/plan.hpp"
#include <iostream>
#include <fstream>
#include <string>


using namespace prx;

int main(int argc, char** argv)
{
    std::string params_file;
    if (argc<=1)
    {
        params_file = "examples/mujoco/mushr_playback.yaml";
    }
    else
    {
        params_file = std::string(argv[1]);
    }
    param_loader params(params_file);

    std::string controlFilename = params["control_file"].as<std::string>();
    std::string modelFilename = params["model_file"].as<std::string>();

    std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>(modelFilename);
    sim->init_simulator();

    auto context = sim -> get_context("mujoco");
    auto ss = context.first -> get_state_space();
    auto cs = context.first -> get_control_space();
    
    init_random(params["random_seed"].as<int>());


    
    space_point_t start = ss ->make_point();
    ss -> copy_point_from_vector(start, params["start_state"].as<std::vector<double>>());
    ss -> copy_from_point(start);
    space_point_t end = ss -> make_point();
    trajectory_t traj(ss);

    plan_t plan(cs);

    std::string line;
    std::ifstream FileReader(input_path + controlFilename);
    std::cout<<"Reading plan file: " <<input_path + controlFilename << "\n";
    while (getline (FileReader, line)) 
    {
        std::vector<double> line_data = split_to_dbl_vector(line);
        //std::cout<<"Read line from plan: "<< line << "\n";
        
        //read time from start of line
        double time = line_data[0];
        plan.append_onto_back(time);
        line_data.erase(line_data.begin());

        //read controls
        space_point_t u = plan.back().control;
        cs -> copy_point_from_vector(u, line_data);

    }
    FileReader.close();
    std::cout<<"Finished reading file\n";

    //execute loaded plan

    context.first -> propagate(start, plan, traj);
    usleep(int(1e6));  

    std::ofstream out ("plot generator/output.txt");
    out<<traj.print();
    out.close();
}