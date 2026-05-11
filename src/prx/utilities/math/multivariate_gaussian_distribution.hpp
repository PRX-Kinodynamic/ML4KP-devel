#pragma once

#include <functional>
#include <random>
#include <gtsam/base/Lie.h>
#include "prx/utilities/general/transforms.hpp"

namespace prx
{

template <int Dimension>
class multivariate_gaussian_t
{
public:
  using Mean = Eigen::Vector<double, Dimension>;
  using Sample = Eigen::Vector<double, Dimension>;
  using Covariance = Eigen::Matrix<double, Dimension, Dimension>;

  multivariate_gaussian_t() : multivariate_gaussian_t(Mean::Zero(), Covariance::Identity())
  {
  }

  multivariate_gaussian_t(Covariance covar) : multivariate_gaussian_t(Mean::Zero(), covar)
  {
  }
  multivariate_gaussian_t(Mean mean, Covariance covar) : _mean(mean)
  {
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
    _sample = _sample.unaryExpr(uniform_sample);
    return _mean + _transform * _sample;  // Mean{ _mean.size() }.unaryExpr([&](auto x) { return dist(gen); });
  }

  Mean operator()() const
  {
    return sample();
  }

protected:
  Mean _mean;
  Covariance _transform;

  // Auxiliary objects which change in const functions but don't matter for the object
  mutable Mean _sample;
  mutable std::mt19937 gen{ std::random_device{}() };
  mutable std::normal_distribution<> dist;

  void compute_transform(const Covariance& cov)
  {
    Eigen::SelfAdjointEigenSolver<Covariance> eigenSolver(cov);
    _transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  std::function<double(double)> uniform_sample = [&](const double x) { return dist(gen); };
};

template <typename LieType>
class lie_group_gaussian_noise_t
{
  static constexpr Eigen::Index Dimension{ gtsam::traits<LieType>::dimension };

  multivariate_gaussian_t<Dimension> _gaussian;

public:
  using Mean = Eigen::Vector<double, Dimension>;
  using Covariance = Eigen::Matrix<double, Dimension, Dimension>;

  lie_group_gaussian_noise_t() : lie_group_gaussian_noise_t(Mean::Zero(), Covariance::Identity())
  {
  }

  lie_group_gaussian_noise_t(Covariance covar) : lie_group_gaussian_noise_t(Mean::Zero(), covar)
  {
  }

  lie_group_gaussian_noise_t(Mean mean, Covariance covar) : _gaussian(mean, covar)
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