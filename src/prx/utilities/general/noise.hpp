#pragma once

#include <random>
#include <algorithm>

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/general/random.hpp"

namespace prx
{
	template<class RandomNumberDistribution> class noise_t;

	typedef noise_t<std::normal_distribution<double>> gaussian_noise_t;
	typedef noise_t<std::uniform_real_distribution<double>> uniform_noise_t;

	template<class RandomNumberDistribution>
	class noise_t
	{
		public:
			noise_t() : noise_t(global_generator)
			{
				// generator = global_generator;
			}

			template<class... Types>
			noise_t(Types... args) : rnd(args...)
			{
			}

			noise_t(std::mt19937_64 _gen)
			{
				// rnd = _rnd;
				generator = _gen;
			}

			void add_noise(const space_point_t& pt, unsigned int start = 0, unsigned int end = std::numeric_limits<unsigned int>::max())
			{
				end = std::min(end, pt -> get_dim());
				for (int i = 0; i < pt -> get_dim(); ++i)
				{
					(*pt)[i] += rnd(generator);
				}
			}

			template<class T, typename = std::enable_if_t<std::is_arithmetic<T>::value> >
			T add_noise(T& val)
			{
				val += rnd(generator);
			}

			template<typename T> // Add template checks to generalize to containers
			void add_noise(std::vector<T>& container)
			{
				for (int i = 0; i < container.size(); ++i)
				{
					container[i] += rnd(generator);
				}
			}

			void add_noise(const Eigen::Ref<const Eigen::MatrixXd> mat)
			{
				// There might be a better (faster) way of doing this
				for (int i = 0; i < mat.rows(); ++i)
				{
					for (int j = 0; j < mat.cols(); ++j)
					{
						mat(i,j) += rnd(generator);
					}
				}
			}

		protected:
			std::mt19937_64 generator;
			RandomNumberDistribution rnd;
	};
}