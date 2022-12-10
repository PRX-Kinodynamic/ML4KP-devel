#include "prx/mujoco/mj_simulator.hpp"
#include "prx/utilities/general/timer.hpp"

using namespace prx;

int main(int argc, char** argv)
{
    mujoco_simulator_t sim("cartpole.xml");
    timer_t timer;
    for (double time = 0; time < 10; time += simulation_step)
    {
        sim.step_simulation(propagate_step::FIRST_STEP);
    }
    std::cout << "Took " << timer.measure() << " seconds to run 10s of simulation." << std::endl;
}