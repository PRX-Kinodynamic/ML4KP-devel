/**
 * @file random.hpp
 * @brief <b> A few helper functions for random sampling. </b>
 * */
#pragma once

#include <vector>
#include <random>

namespace prx
{
namespace random
{
static std::uniform_real_distribution<double> uniform_zero_one(0.0,
                                                               std::nextafter(1.0, std::numeric_limits<double>::max()));
static std::normal_distribution<double> gaussian_zero_one(0.0, 1.0);
}  // namespace random
// This might not be the best way to have a generator
extern std::mt19937_64 global_generator;

/**
 * Initializes the uniform random number generator with the given seed.
 *
 * @brief Initializes the uniform random number generator with the given seed.
 * @param seed The value to seed the RNG with.  Default is 10.
 * @author Zakary Littlefield
 */
void init_random(int seed);

/**
 * Returns a random number from the uniform distribution [0,1).
 *
 * @brief Returns a random number from the uniform distribution [0,1).
 * @author Zakary Littlefield
 *
 * @return A double precision random number.
 */
double uniform_random();

/**
 * Returns a random number from a Gaussian Distribution
 *
 * @brief Returns a random number from a Gaussian Distribution.
 * @author Zakary Littlefield
 * @return A double precision random number.
 */
template <typename Type = double>
Type gaussian_random(const Type mean = 0.0, const Type stddev = 1.0)
{
  const double x{ ::prx::random::gaussian_zero_one(global_generator) };
  return x * stddev + mean;
}

/**
 * Returns a random number from the uniform distribution within the
 * given range.
 *
 * @brief Returns a random number from the uniform distribution within the given range.
 * @param min The minimum random value to return.
 * @param max The maximum random value to return.
 * @author Zakary Littlefield, Edgar Granados
 *
 * @return A double precision random number in the given range.
 */
template <typename Type>
double uniform_random(const Type min, const Type max)
{
  const double val{ ::prx::random::uniform_zero_one(global_generator) * (max - min) + min };
  return val;
}

/**
 * Returns a random integer number from the uniform distribution within
 * the given closed range [min,max]. Both max and min are possible return values
 *
 * @brief Returns a random integer number from the uniform distribution within the given closed range.
 * @param min The minimum random value to return.
 * @param max The maximum random value to return.
 * @author Zakary Littlefield
 *
 * @return An integer random number in the given range.
 */
int uniform_int_random(int min, int max);

//
template <typename Container, typename Type>
void uniform_random(Container& container, const Type min, const Type max)
{
  for (int i = 0; i < container.size(); ++i)
  {
    container[i] = uniform_random(min, max);
  }
}

template <typename Container, typename Type, typename... Initializers>
Container uniform_random(const Type min, const Type max, Initializers... args)
{
  Container container{ args... };
  for (int i = 0; i < container.size(); ++i)
  {
    container[i] = uniform_random(min, max);
  }
  return container;
}

template <typename Container, typename Type, typename... Initializers>
Container gaussian_random(const Type min, const Type max, Initializers... args)
{
  Container container{ args... };
  for (int i = 0; i < container.size(); ++i)
  {
    container[i] = gaussian_random(min, max);
  }
  return container;
}

/**
 * Given a set of weights, randomly roll a "dice" and obtain an event
 *
 * For example, given a probability distribution {10%,10%,80%} this function
 * will roll a dice weighted with those probabilities and return the index
 * of the event that occurred.
 *
 * @brief Given a set of weights, randomly roll a "dice" and obtain an event
 * @param weights A probability distribution signifying how likely an event will occur
 * @authors Andrew Kimmel, Nick Stiffler
 *
 * @return The index of the weighted event that happened
 */
int roll_weighted_die(std::vector<double> const& weights);

}  // namespace prx