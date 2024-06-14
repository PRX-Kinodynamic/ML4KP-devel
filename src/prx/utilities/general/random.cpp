#include "prx/utilities/general/random.hpp"
#include <cmath>
#include <cstdlib>

namespace prx
{
std::mt19937_64 global_generator;

void init_random(int seed)
{
  srand(seed);
  global_generator.seed(seed);
}

double uniform_random()
{
  // double val = rand()*1.0 / RAND_MAX;
  // double val = static_cast<double>(global_generator()) / static_cast<double>(global_generator.max());
  // double val = global_generator()*1.0 / global_generator.max();
  const double val = ::prx::random::uniform_zero_one(global_generator);
  return val;
}

double uniform_random(double min, double max)
{
  // TODO: This implementation is problematic if (max - min) > std::::numeric_limits<double>::max()
  // ==> change to use std::uniform_real_distribution?
  // double val = (((double)rand() / (double)RAND_MAX) * (max - min)) + min;
  const double val = ::prx::random::uniform_zero_one(global_generator) * (max - min) + min;
  // double val = ((static_cast<double>(global_generator()) / static_cast<double>(global_generator.max())) * (max -
  // min)) + min; std::uniform_real_distribution<double> dist(min, max); return dist(global_generator);
  return val;
}

int uniform_int_random(int min, int max)
{
  const double raw_rand{ ::prx::random::uniform_zero_one(global_generator) };
  const int val{ static_cast<int>(std::floor(raw_rand * (max + 1 - min) + min)) };
  return val;
}

int roll_weighted_die(std::vector<double> const& weights)
{
  int event_index = -1;

  double sum = 0;
  for (unsigned i = 0; i < weights.size(); i++)
  {
    sum += weights[i];
  }
  double val = uniform_random();
  double running_total = 0;
  for (unsigned i = 0; i < weights.size(); i++)
  {
    running_total += weights[i] / sum;
    if (val <= running_total)
    {
      event_index = i;
      break;
    }
  }
  return event_index;
}

double gaussian_random(const double mean, const double stddev)
{
  const double x{ ::prx::random::gaussian_zero_one(global_generator) };
  return x * stddev + mean;
}
}  // namespace prx
