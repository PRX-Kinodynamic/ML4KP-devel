#pragma once

#include <functional>
#include <random>
#include <gtsam/base/Lie.h>
#include "prx/utilities/general/transforms.hpp"

namespace prx
{

// For the bounded gaussian, the input chi2 expects the value from the chi2 table for Dimension and 1-alpha.
// For a 1dim gaussian with 95% confidence (0.05 column), the input value must be 3.841.
// For a 2dim gaussian with 95% confidence (0.05 column), the input value must be 5.991.
// The bounded implementation can be *slow* (rejects samples out of bounds). A faster implementation should be possible
template <int Dimension>
class multivariate_gaussian_t
{
public:
  using Mean = Eigen::Vector<double, Dimension>;
  using Sample = Eigen::Vector<double, Dimension>;
  using Covariance = Eigen::Matrix<double, Dimension, Dimension>;

  multivariate_gaussian_t(const bool bounded = false, const double chi2 = 0.0)
    : multivariate_gaussian_t(Mean::Zero(), Covariance::Identity(), bounded, chi2)
  {
  }

  multivariate_gaussian_t(Covariance covar, const bool bounded = false, const double chi2 = 0.0)
    : multivariate_gaussian_t(Mean::Zero(), covar, bounded, chi2)
  {
  }
  multivariate_gaussian_t(Mean mean, Covariance covar, const bool bounded = false, const double chi2 = 0.0)
    : _mean(mean), _bounded(bounded), _chi2_confidence(std::sqrt(chi2))
  {
    if (_bounded)
    {
      prx_assert(_chi2_confidence > 0, "[multivariate_gaussian_t] Need confidence level for bounded gaussian ");
    }
    compute_transform(covar);
  }

  void set(const Covariance& covar)
  {
    compute_transform(covar);
  }
  void set(const Mean& mean, const Covariance& covar)
  {
    _mean = mean;
    compute_transform(covar);
  }

  Mean sample() const
  {
    // _sample = _sample.unaryExpr(gaussian_sample);
    // _sample = Mean::NullaryExpr(gaussian_sample);
    sample_01();
    return _mean + _transform * _sample;  // Mean{ _mean.size() }.unaryExpr([&](auto x) { return dist(gen); });
  }

  Mean operator()() const
  {
    return sample();
  }

  Mean operator()(const Mean& x) const
  {
    return x + sample();
  }

protected:
  const bool _bounded;
  const double _chi2_confidence;
  Mean _mean;
  Covariance _transform;

  // Auxiliary objects which change in const functions but don't matter for the object
  mutable Mean _sample;
  mutable std::mt19937 gen{ std::random_device{}() };
  mutable std::normal_distribution<> dist;

  void compute_transform(const Covariance& cov)
  {
    if (cov.isZero(1e-6))
    {
      _transform = Covariance::Zero();
    }
    else
    {
      Eigen::SelfAdjointEigenSolver<Covariance> eigenSolver(cov);
      const Eigen::Matrix<double, Dimension, Dimension> eig_vecs{ eigenSolver.eigenvectors() };
      const Eigen::Vector<double, Dimension> eig_vals{ eigenSolver.eigenvalues() };

      _transform = eig_vecs * eig_vals.cwiseSqrt().asDiagonal();
    }
  }
  // 2.447651936
  bool in_bounds() const
  {
    const double rad{ _sample.norm() };
    return rad < _chi2_confidence;
  }
  void sample_01() const
  {
    _sample = Mean::NullaryExpr(gaussian_sample);
    while (_bounded and not in_bounds())
    {
      _sample = Mean::NullaryExpr(gaussian_sample);
    }
  }

  std::function<double()> gaussian_sample = [&]() {
    // double si{ dist(gen) };
    // while (not(-1.96 <= si and si <= 1.96))
    // {
    //   si = dist(gen);
    // }
    return dist(gen);
  };
  // std::function<double()> gaussian_bounded_sample = [&]() { return u_dist(gen); };
};

template <typename LieType>
class lie_group_gaussian_noise_t
{
  static constexpr Eigen::Index Dimension{ gtsam::traits<LieType>::dimension };

  multivariate_gaussian_t<Dimension> _gaussian;

public:
  using Mean = Eigen::Vector<double, Dimension>;
  using Covariance = Eigen::Matrix<double, Dimension, Dimension>;

  lie_group_gaussian_noise_t(const bool bounded = false, const double chi2 = 0.0)
    : lie_group_gaussian_noise_t(Mean::Zero(), Covariance::Identity(), bounded, chi2)
  {
  }

  lie_group_gaussian_noise_t(Covariance covar, const bool bounded = false, const double chi2 = 0.0)
    : lie_group_gaussian_noise_t(Mean::Zero(), covar, bounded, chi2)
  {
  }

  lie_group_gaussian_noise_t(Mean mean, Covariance covar, const bool bounded = false, const double chi2 = 0.0)
    : _gaussian(mean, covar, bounded, chi2)
  {
  }

  void set(const Covariance& covar)
  {
    _gaussian.set(covar);
  }
  void set(const Mean& mean, const Covariance& covar)
  {
    _gaussian.set(mean, covar);
  }

  LieType operator()(const LieType& lie) const
  {
    const Mean sample{ _gaussian() };
    const LieType xsample{ gtsam::traits<LieType>::Expmap(sample) };
    return gtsam::traits<LieType>::Compose(lie, xsample);
  }
};

}  // namespace prx