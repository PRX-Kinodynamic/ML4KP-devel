#include "pyprx/simulation/plants/two_link_acrobot_py.hpp"
#include "pyprx/simulation/plants/two_dimensional_point_py.hpp"
#include "pyprx/simulation/plants/types/types_py.hpp"
#include "pyprx/simulation/plants/pendulum_py.hpp"

void pyprx_simulation_plants()
{
	pyprx_simulation_plants_types();

   	pyprx_simulation_plants_2DPT_py();
	pyprx_simulation_plants_acrobot_py();
   	// pyprx_simulation_plants_acrobot();
   	pyprx_simulation_plants_pendulum();
}
