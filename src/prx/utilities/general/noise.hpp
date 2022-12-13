#pragma once

#include <random>
#include <algorithm>

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/general/random.hpp"

namespace prx
{
template <class RandomNumberDistribution>
class noise_t;

/**
 * Gaussian noise (mean=0.0, std_dev=1.0)
 */
typedef noise_t<std::normal_distribution<double>> gaussian_noise_t;

/**
 * Uniform noise, must provide (a,b) to constructor.
 */
typedef noise_t<std::uniform_real_distribution<double>> uniform_noise_t;

template <class RandomNumberDistribution>
class noise_t
{
public:
  // noise_t() //: generator(global_generator)
  // {
  // 	generator = global_generator
  // }

  template <class... Types>
  noise_t(Types... args) : generator(global_generator), rnd(args...)
  {
    // generator = global_generator;
  }

  // noise_t(global_generator) : generator(global_generator)
  // {
  // }

  void set_generator(std::mt19937_64 _gen)
  {
    // rnd = _rnd;
    generator = _gen;
  }

  void add_noise(const space_point_t& pt, unsigned int start = 0,
                 std::size_t end = std::numeric_limits<unsigned int>::max()) const
  {
    end = std::min(end, pt->get_dim());
    for (int i = 0; i < pt->get_dim(); ++i)
    {
      (*pt)[i] += rnd(generator);
    }
  }

  template <class T, typename = std::enable_if_t<std::is_arithmetic<T>::value>>
  T add_noise(const T val) const
  {
    return val + rnd(generator);
  }

  void add_noise(const space_t* space) const
  {
    for (int i = 0; i < space->size(); ++i)
    {
      space->at(i) = add_noise(space->at(i));
    }
  }

  template <typename T>  // Add template checks to generalize to containers
  void add_noise(std::vector<T>& container) const
  {
    for (int i = 0; i < container.size(); ++i)
    {
      container[i] += rnd(generator);
    }
  }

  void add_noise(Eigen::Ref<Eigen::MatrixXd> mat) const
  {
    // There might be a better (faster) way of doing this
    for (int i = 0; i < mat.rows(); ++i)
    {
      for (int j = 0; j < mat.cols(); ++j)
      {
        mat(i, j) += rnd(generator);
      }
    }
  }

protected:
  mutable std::mt19937_64 generator;
  mutable RandomNumberDistribution rnd;
};
}  // namespace prx