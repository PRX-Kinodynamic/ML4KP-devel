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
        params_file = "examples/mujoco/cartpole_playback.yaml";
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
    space_point_t end = ss -> make_point();

    plan_t plan(cs);

    std::string line;
    std::ifstream FileReader("resources/input_files/plans/"+controlFilename);
    while (getline (FileReader, line)) {
        
        //read time from start of line
        int startindx = 0;
        int endindx = line.find(",");
        double time = std::stod(line.substr(startindx, endindx - startindx));
        startindx = endindx + 1;
        plan.append_onto_back(time);


        std::cout<<" ";
        
        //read controls
        space_point_t u = plan.back().control;
        for(int i = 0; i< u->get_dim();i++){
            if(endindx != -1){
                endindx = line.find(",", startindx);
                u->at(i) = std::stod(line.substr(startindx, endindx - startindx));
                startindx = endindx + 1;
            }else{
                prx_throw("WARNING: control space larger than input control!\n");
            }
        }
    }
    FileReader.close();

    //execute loaded plan
    while(true)
    {
        context.first -> propagate(start, plan, end);
        usleep(int(1e6));
    }
}