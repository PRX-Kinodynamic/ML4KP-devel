#pragma once

#include <bitset>
#include <iostream>
#include <bit>
#include <cmath>
#include <queue>
#include <map>
#include <algorithm>
namespace prx
{
class chi_squared
{
public:
  chi_squared(const double tolerance) : _tolerance(tolerance), _cmp_function(tolerance)
  {
  }

  chi_squared() : chi_squared(0.0001) {};

  double fy(const int v, const double u) const
  {
    const double v2{ v / 2. };
    const double eu2{ std::exp(-u / 2) };

    if (v == 1)
    {
      return std::erf(std::sqrt(u));
    }
    const double f0{ 1. / (std::pow(2, v2) * std::tgamma(v2)) };
    const double f1{ std::pow(u, std::max(v2 - 1, 0.)) * eu2 };

    return f0 * f1;
  }

  double critical_value(const int degree, const double alpha)
  {
    if (_cache.count(degree) > 0)
    {
      const double p{ 1. - alpha };
      CacheProbability& prob_cache{ _cache[degree] };

      auto iter = find(prob_cache, p);
      // auto iter = std::upper_bound(prob_cache.begin(), prob_cache.end(), p, _cmp_function);
      // const bool alpha_computed{) };
      if (_cmp_function.equal(std::get<0>(*iter), p))
      {
        return std::get<1>(*iter);
      }
      else if (_cmp_function.equal(std::get<0>(*(iter + 1)), p))
      {
        return std::get<1>(*(iter + 1));
      }
      else
      {
        // if (prob_cache.size() > 1)
        // {
        //   iter = iter + 1;
        // }
        // if (iter == prob_cache.end())
        // {
        //   iter = prob_cache.end() - 1;
        // }
        const double y{ std::get<1>(*iter) };
        const double sum{ std::get<2>(*iter) };
        const AlphaProbSum chi2_val{ chi2(degree, p, sum, y) };
        prob_cache.insert(iter + 1, chi2_val);
        return std::get<1>(chi2_val);
      }
    }
    CacheProbability deg_cache;
    const AlphaProbSum chi2_val{ chi2(degree, 0, 0, 0) };
    deg_cache.push_back(chi2_val);
    _cache[degree] = deg_cache;

    return critical_value(degree, alpha);

    // _cache.find()
  }

private:
  // {1-alpha, chi2_val, probability}
  using AlphaProbSum = std::tuple<double, double, double>;
  using CacheProbability = std::vector<AlphaProbSum>;  //, std::vector<UProbPair>, cmp_t>;
  using CacheDegree = std::map<int, CacheProbability>;
  using Iterator = CacheProbability::iterator;

  struct cmp_t
  {
    // comp	-	binary predicate which returns true if the first argument is ordered before the second.
    cmp_t(const double tolerance) : _tolerance(tolerance) {};
    bool operator()(const AlphaProbSum& z1, const AlphaProbSum& z2) const
    {
      // return std::fabs(z1.first - z2.first) < _tolerance;
      return cmp(std::get<0>(z1), std::get<0>(z2));
    };
    bool operator()(const AlphaProbSum& z1, const double& z2) const
    {
      return cmp(std::get<0>(z1), z2);
    }
    bool operator()(const double z1, const AlphaProbSum& z2) const
    {
      return cmp(z1, std::get<0>(z2));
    }

    bool cmp(const double& z1, const double& z2) const
    {
      // PRX_DBG_VARS(z1, z2);
      return z1 < z2;
    };
    bool equal(const double& z1, const double& z2) const
    {
      return std::fabs(z1 - z2) < _tolerance;
    }
    double _tolerance;
  };

  Iterator find(CacheProbability& container, const double& p)
  {
    // auto iter = std::upper_bound(prob_cache.begin(), prob_cache.end(), p, _cmp_function);
    auto prev_iter = container.begin();
    for (auto iter = container.begin(); iter != container.end(); iter += 1)
    {
      if (_cmp_function(*iter, p))
      {
        prev_iter = iter;
      }
      else if (_cmp_function.equal(std::get<0>(*iter), p))
      {
        return iter;
      }
      else
      {
        return prev_iter;
      }
    }
    return prev_iter;
  }

  AlphaProbSum chi2(const int v, const double p, const double sum_init, const double u_init = 0)
  {
    double sum{ sum_init };
    double du{ _tolerance };
    double u{ v == 1 ? u_init / 2 : u_init };

    while (sum < p)
    {
      if (sum > 1)
      {
        std::cerr << "Error: " << sum << std::endl;
        break;
      }
      if (v == 1)
      {
        sum = fy(v, u);
      }
      else
      {
        sum += fy(v, u) * du;
      }
      u += du;
    }
    if (v == 1)
    {
      u *= 2;
    }
    return { p, u, sum };
  }

  double _tolerance;
  cmp_t _cmp_function;
  CacheDegree _cache;
};
}  // namespace prx
// int main()
// {
//     fy(1, 0.0);
//     // std::cout << "erf: " << std::erf(std::sqrt(0)) ;
//     // std::cout << "fy: " << fy(1, 0.0) << "\n";
//     std::cout << "chi: " << chi2(1, 0.995);
//     return 0;
// }