#include "prx/mujoco/mj_simulator.hpp"

using namespace prx;

int main(int argc, char** argv)
{
    mujoco_simulator_t sim("cartpole.xml");
    for (double time = 0; time < 10; time += simulation_step)
    {
        sim.step_simulation(propagate_step::FIRST_STEP);
    }
}