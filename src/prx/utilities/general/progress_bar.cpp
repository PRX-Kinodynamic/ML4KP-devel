#include "prx/utilities/general/progress_bar.hpp"

namespace prx
{		

	progress_bar_t::progress_bar_t(double _total, std::string _label)
	{
		total = _total;
		label = _label;
		next_current = -1;
	}

	void progress_bar_t::update(double current_value)
	{
		if (next_current == -1)
		{
			std::cout << "0%   10   20   30   40   50   60   70   80   90  100%" << std::endl;
			std::cout << "[--------------------------------------------------]" << std::endl;
			next_current = 2.0 * total / 100.0;
			// next_current = current_value;
			// return;
		}
		if ( current_value < next_current ) return;

		int i = 0;
		std::cout << label;
		std::cout << "[";
		for (; i < 100 * current_value / total; i+=2)
		{
			std::cout << "*";
		}
		for (; i < 100; i+=2)
		{
			std::cout << " ";
		}
		std::cout << "]" << std::endl;

		next_current += 2.0 * total / 100.0;

	}

}