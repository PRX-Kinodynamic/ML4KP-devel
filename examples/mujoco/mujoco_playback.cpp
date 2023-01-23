#include "prx/mujoco/mj_simulator.hpp"
#include "prx/simulation/playback/plan.hpp"
#include <iostream>
#include <fstream>
#include <string>

using namespace prx;

// arg 1: file name of target control file in resources/control_sequences/
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
        // Output the text from the file
        std::cout << line;
        
        int start = 0;
        int end = line.find(",");
        double time = std::stod(line.substr(start, end - start));
        int start = end + 1;

        plan.append_onto_back(time);

        space_point_t u = plan.back().control;
        for(int i = 0; i< u->get_dim();i++){
            end = line.find(",", start);
            if(end != -1){
                u->at(i) = std::stod(line.substr(start, end - start));
                start = end + 1;
            }
        }
    }
    FileReader.close();

    while(true)
    {
        context.first -> propagate(start, plan, end);
        usleep(int(1e6));
    }
}