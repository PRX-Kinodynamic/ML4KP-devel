#pragma once

#include "prx/simulation/controller.hpp"
#include "prx/simulation/plants/types/linear_time_invariant.hpp"
#include "prx/utilities/math/continuous_algebraic_riccati_equation.hpp"

namespace prx
{
	class bang_bang_t : public controller_t
	{
		public:
		bang_bang_t(system_ptr_t _plant, std::string _name = "bang-bang_ctrl")
			: controller_t(_plant, _name)
		{
			ctrl_num = 0;
			auto cs = plant -> get_control_space();
			auto lb = cs -> get_lower_bounds();
			auto ub = cs -> get_upper_bounds();
			std::vector<double> aux(lb.begin(), lb.end());
			// TODO: add ctrls with 0s
			for (int i = 0; i < std::pow(2, cs -> get_dimension()); ++i)
			{
				// for (int j = 1; j < std::pow(2, cs -> get_dimension()); j = j << 1)
				int k = 1;
				for (int j = 0; j < cs -> get_dimension(); ++j)
				{
					// printf("i: %d\tj: %d\tmodulo: %d\n", i, j,i % static_cast<int>(std::pow(2, j)) );
					// printf("i: %d\tj: %d\tAND: %d\n", i, j, i & j);
					// if ( i % static_cast<int>(std::pow(2, j)) )
					// if (aux[j] == lb[j])
					if ( (i & k) )
					{
						aux[j] = ub[j];
					}
					else
					{
						aux[j] = lb[j];
					}
					k = k << 1;
				}
				auto pt = cs -> make_point();
				cs -> copy_point_from_vector(pt, aux);
				ctrls.push_back(pt);
				std::cout << "ctrl: " << pt << std::endl;
				// PRX_DEBUG_ITERABLE("AUX", aux)
			}
		}

		virtual ~bang_bang_t(){}

		virtual void compute_controls() override
		{
			auto ss = plant -> get_control_space();
		}

		space_point_t get_control_at(unsigned int i)
		{
			prx_assert(i < ctrls.size(), "Requested control " << i << "but only got " << ctrls.size() << " controls");
			return ctrls[i];
		}

		unsigned int get_num_ctrls()
		{
			return ctrls.size();
		}

		protected:
		unsigned int ctrl_num;
		std::vector<space_point_t> ctrls;

	};
}