#pragma once

#include <vector>
#include <numeric>
#include "prx/utilities/general/zipped_iter.hpp"

namespace prx
{
	class progress_bar_t
	{
		public:
			progress_bar_t(double total, std::string label = "");
			~progress_bar_t() = default;

			void update(double current_value);

		private:
			double total;
			double next_current;
			std::string label;

	};
}