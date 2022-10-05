#include "pyprx/simulation/plants/types/linear_time_invariant_py.hpp"
#include "pyprx/simulation/plants/types/linear_time_variant_py.hpp"
#include "pyprx/simulation/plants/types/noisy_plant_py.hpp"

void pyprx_simulation_plants_types()
{
	pyprx_simulation_plants_types_lti();
	pyprx_simulation_plants_types_ltv();
	pyprx_simulation_plants_types_noisy_plant();
}
