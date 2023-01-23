#include "prx/mujoco/mj_simulator.hpp"
#include "prx/simulation/playback/plan.hpp"
#include <iostream>
#include <fstream>
#include <string>

using namespace prx;

// arg 1: file name of target control file in resources/control_sequences/
// arg 2: file name of target model file in resources/models/mujoco/
int main(int argc, char** argv)
{

    std::string controlFilename(argv[1]);
    std::string modelFilename(argv[2]);

    std::shared_ptr<mujoco_simulator_t> sim = std::make_shared<mujoco_simulator_t>(modelFilename);
    sim->init_simulator();

    auto context = sim -> get_context("mujoco");
    auto ss = context.first -> get_state_space();
    auto cs = context.first -> get_control_space();

    init_random(111093);
    space_point_t start = ss -> make_point();
    space_point_t end = ss -> make_point();
    start -> at(2) = PRX_PI;

    plan_t plan(cs);


    std::string line;
    std::ifstream FileReader("resources/control_sequences/"+controlFilename);
    while (getline (FileReader, line)) {
        
        //read time from start of line
        int start = 0;
        int end = line.find(",");
        double time = std::stod(line.substr(start, end - start));
        start = end + 1;
        plan.append_onto_back(time);

        //read controls
        space_point_t u = plan.back().control;
        for(int i = 0; i< u->get_dim();i++){
            if(end != -1){
                end = line.find(",", start);
                u->at(i) = std::stod(line.substr(start, end - start));
                start = end + 1;
            }else{
                std::cout<< "WARNING: control space larger than input control!\n";
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