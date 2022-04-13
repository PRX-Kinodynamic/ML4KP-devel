#pragma once
#include <fstream>

namespace prx
{
	// TODO: is this the best (faster) way of logging?
	class logger_t
	{	
		public:
			logger_t(const std::string& file_name, char separator = ' ')
			{
				ofs_logger.open(file_name.c_str(), std::ofstream::out | std::ofstream::trunc);
				sep = separator;
			}

			virtual ~logger_t()
			{
				ofs_logger.flush();
				ofs_logger.close();
			}

			template <typename T>
			void add_values(T values)
			{
				for(auto v : values)
				{
					ofs_logger << v << sep;
				}
				ofs_logger << '\n';
			}

		// TODO: overwrite operator<<


		protected:
			std::ofstream ofs_logger;
			char sep;
	};

}