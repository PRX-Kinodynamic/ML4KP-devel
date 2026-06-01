#pragma once
#include "prx/utilities/general/debug_utils.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/simulation/playback/piecewise_plan.hpp"
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/ProductLieGroup.h>
#include <type_traits>
namespace prx
{

// template <typename T>  // primary template
template <class T>
class sampler_t
{
public:
  T operator()()
  {
    PRX_NOT_IMPLEMENTED;
  }
  std::deque<T> _buffer;
};

template <>  // primary template
class sampler_t<double>
{
public:
  using Sampler = sampler_t<double>;
  using Bounds = double;

  sampler_t(const double min, const double max) : _min(min), _max(max)
  {
  }
  sampler_t() : sampler_t(-1., 1.)
  {
  }
  sampler_t(prx::param_loader param) : sampler_t(param.get_or_default("min", -1.0), param.get_or_default("max", 1.0))
  {
  }

  double operator()()
  {
    return uniform_random(_min, _max);
  }

  void bounds(const Bounds min, const Bounds max)
  {
    _min = min;
    _max = max;
  }

  std::pair<Bounds, Bounds> bounds() const
  {
    return { _min, _max };
  }

  friend std::ostream& operator<<(std::ostream& os, const Sampler& obj)
  {
    os << "min: ";
    os << obj._min << "\n";
    os << "max: ";
    os << obj._max << "\n";
    return os;
  }

private:
  double _min, _max;
};

template <>  // primary template
class sampler_t<int>
{
public:
  using Sampler = sampler_t<int>;
  using Bounds = int;

  sampler_t(const int min, const int max) : _min(min), _max(max)
  {
  }
  sampler_t() : sampler_t(-1, 1)
  {
  }
  sampler_t(prx::param_loader param) : sampler_t(param.get_or_default("min", -1), param.get_or_default("max", 1))
  {
  }

  int operator()()
  {
    return uniform_int_random(_min, _max);
  }

  void bounds(const Bounds min, const Bounds max)
  {
    _min = min;
    _max = max;
  }

  friend std::ostream& operator<<(std::ostream& os, const Sampler& obj)
  {
    os << "min: ";
    os << obj._min << "\n";
    os << "max: ";
    os << obj._max << "\n";
    return os;
  }

private:
  int _min, _max;
};

// true if sample > cut, false otherwise. 0 <= cut < 1
template <>  // primary template
class sampler_t<bool>
{
public:
  using Sampler = sampler_t<bool>;
  using Bounds = double;

  sampler_t(const Bounds cut) : _sampler(0., 1.), _cut(cut)
  {
  }
  sampler_t() : sampler_t(0.5)
  {
  }
  sampler_t(prx::param_loader param) : sampler_t(param.get_or_default("cut", 0.5))
  {
  }

  int operator()()
  {
    return _sampler() > _cut;
  }

  void bounds(const Bounds min, const Bounds max)
  {
    _sampler.bounds(min, max);
  }

  friend std::ostream& operator<<(std::ostream& os, const Sampler& obj)
  {
    os << "cut: ";
    os << obj._cut << "\n";
    return os;
  }

private:
  double _cut;
  sampler_t<double> _sampler;
};

template <typename Type>
class sampler_t<std::vector<Type>>
{
  using TypeSampler = sampler_t<Type>;

public:
  using Sampler = sampler_t<std::vector<Type>>;
  using Element = std::vector<Type>;
  using Bounds = std::vector<Type>;
  // using Bounds = std::vector<double, Dim>;

  sampler_t(const Bounds min, const Bounds max)  //: _min(min), _max(max), _diff(max - min)
  {
    bounds(min, max);
  }

  sampler_t() : _samplers()
  {
  }

  sampler_t(prx::param_loader param)
    : sampler_t(param.get_or_default("min", Element()), param.get_or_default("max", Element()))
  {
    // TODO
    // std::vector<Type> min, max;
    // for (int i = 0; i < min.size(); ++i)
    // {
    //   _samplers.emplace_back(min[i], max[i]);
    // }
  }

  Element operator()()
  {
    Element samples;
    for (auto&& sampler : _samplers)
    {
      samples.push_back(sampler());
    }
    return samples;
  }

  void bounds(const Bounds min, const Bounds max)
  {
    for (int i = 0; i < min.size(); ++i)
    {
      _samplers.emplace_back(min[i], max[i]);
    }
  }

  friend std::ostream& operator<<(std::ostream& os, const Sampler& obj)
  {
    for (auto& sampler : obj._samplers)
    {
      os << sampler;
    }
    return os;
  }

protected:
  std::vector<TypeSampler> _samplers;
};

template <int Dim>                           // primary template
class sampler_t<Eigen::Vector<double, Dim>>  //: std::false_type
{
public:
  using Sampler = sampler_t<Eigen::Vector<double, Dim>>;
  using Element = Eigen::Vector<double, Dim>;
  using Bounds = Eigen::Vector<double, Dim>;

  sampler_t(const Element min, const Element max) : _min(min), _max(max), _diff(max - min)
  {
  }
  sampler_t() : sampler_t(-Element::Ones(), Element::Ones())
  {
  }

  sampler_t(prx::param_loader param)
    : sampler_t(param.get_or_default("min", Element()), param.get_or_default("max", Element()))
  {
  }

  Element operator()() const
  {
    const Element random{ Element::Random() };
    return _min + ((random + Element::Ones())).cwiseProduct(_diff * 0.5);
  }

  void bounds(const Element min, const Element max)
  {
    _min = min;
    _max = max;
    _diff = max - min;
  }
  std::pair<Element, Element> bounds() const
  {
    return { _min, _max };
  }

  friend std::ostream& operator<<(std::ostream& os, const Sampler& obj)
  {
    os << "min: ";
    prx::streamer_t<Element>::to_stream(os, obj._min);
    os << "\nmax: ";
    prx::streamer_t<Element>::to_stream(os, obj._max);
    os << "\n";
    return os;
  }

protected:
  Element _min;
  Element _max;
  Element _diff;
};

template <>                   // explicit specialization for T = void
class sampler_t<gtsam::Rot2>  //: std::true_type
{
public:
  using Sampler = sampler_t<gtsam::Rot2>;
  using Bounds = double;

  sampler_t(const Bounds min, const Bounds max) : _aux_sampler(min, max)
  {
  }

  sampler_t() : sampler_t(-prx::constants::pi, prx::constants::pi)
  {
  }

  sampler_t(prx::param_loader params)
    : sampler_t(params.get_or_default("min", -prx::constants::pi), params.get_or_default("max", prx::constants::pi))
  {
  }

  gtsam::Rot2 operator()()
  {
    return std::move(gtsam::Rot2(_aux_sampler()));
  }
  void bounds(const Bounds min, const Bounds max)
  {
    _aux_sampler.bounds(min, max);
  }

  std::pair<Bounds, Bounds> bounds() const
  {
    return _aux_sampler.bounds();
  }

  friend std::ostream& operator<<(std::ostream& os, const Sampler& obj)
  {
    os << obj._aux_sampler;
    return os;
  }

protected:
  sampler_t<Bounds> _aux_sampler;
};

template <>                    // explicit specialization for T = void
class sampler_t<gtsam::Pose2>  //: std::true_type
{
public:
  using Sampler = sampler_t<gtsam::Pose2>;
  using Bounds = Eigen::Vector3d;

  sampler_t(const Bounds min, const Bounds max)
  {
  }

  sampler_t() : sampler_t(Bounds(-1., -1., -prx::constants::pi), Bounds(1., 1., prx::constants::pi))
  {
  }

  sampler_t(prx::param_loader params)
    : sampler_t(params.get_or_default("min", Bounds(-1., -1., -prx::constants::pi)),
                params.get_or_default("max", Bounds(1., 1., prx::constants::pi)))
  {
  }

  gtsam::Pose2 operator()()
  {
    const Bounds random{ _aux_sampler() };
    return std::move(gtsam::Pose2(random[0], random[1], random[2]));
  }
  void bounds(const Bounds min, const Bounds max)
  {
    _aux_sampler.bounds(min, max);
  }

  std::pair<Bounds, Bounds> bounds() const
  {
    return _aux_sampler.bounds();
  }

  friend std::ostream& operator<<(std::ostream& os, const Sampler& obj)
  {
    os << obj._aux_sampler;
    return os;
  }

protected:
  sampler_t<Bounds> _aux_sampler;
};

template <typename G, typename H>
class sampler_t<gtsam::ProductLieGroup<G, H>>
{
  static prx::param_loader process_params(prx::param_loader params, const int idx, const bool car)
  {
    auto iter = params.begin();
    for (int i = 0; i < idx; ++i)
    {
      iter++;
    }
    // YAML::Node& node(*iter);
    prx::param_loader single_params(*iter);
    // PRX_DBG_VARS(single_params)
    iter++;
    auto last_iter = params.end();
    if (iter == last_iter)
    {
      return single_params;
    }
    // Params is a list [car | cdr], where cdr may be a list
    // If car -> return ONLY car, else return the remaining list
    if (car)
    {
      // prx::param_loader res(*iter);
      // PRX_DBG_VARS(res)
      return single_params;
    }
    prx::param_loader res(iter, params.end());
    // PRX_DBG_VARS(res)
    return res;
  }

public:
  using Sampler = sampler_t<gtsam::ProductLieGroup<G, H>>;
  using Element = gtsam::ProductLieGroup<G, H>;
  using GBounds = typename sampler_t<G>::Bounds;
  using HBounds = typename sampler_t<H>::Bounds;
  using Bounds = std::pair<GBounds, HBounds>;

  // template <typename GBounds, typename HBounds>
  sampler_t(const GBounds Gmin, const GBounds Gmax, const HBounds Hmin, const HBounds Hmax)
    : _G_sampler(Gmin, Gmax), _H_sampler(Hmin, Hmax)
  {
  }

  sampler_t() : _G_sampler(), _H_sampler()
  {
  }

  sampler_t(prx::param_loader params) : sampler_t(process_params(params, 0, true), process_params(params, 1, false))
  {
  }

  sampler_t(prx::param_loader Gparams, prx::param_loader Hparams) : _G_sampler(Gparams), _H_sampler(Hparams)
  {
  }

  Element operator()()
  {
    return std::move(Element(_G_sampler(), _H_sampler()));
  }

  void bounds(const GBounds Gmin, const GBounds Gmax, const HBounds Hmin, const HBounds Hmax)
  {
    _G_sampler.bounds(Gmin, Gmax);
    _H_sampler.bounds(Hmin, Hmax);
  }

  std::pair<Bounds, Bounds> bounds()
  {
    return { _G_sampler.bounds(), _H_sampler.bounds() };
  }

  // void bounds(const GBounds Gmin, const GBounds Gmax, const HBounds Hmin, const HBounds Hmax)

  friend std::ostream& operator<<(std::ostream& os, const Sampler& obj)
  {
    prx::streamer_t<sampler_t<G>>::to_stream(os, obj._G_sampler);
    prx::streamer_t<sampler_t<H>>::to_stream(os, obj._H_sampler);
    return os;
  }

protected:
  sampler_t<G> _G_sampler;
  sampler_t<H> _H_sampler;
};

template <typename ControlType, typename DurationType>
class sampler_t<prx::experimental::piecewise_step_t<ControlType, DurationType>>
{
public:
  using PiecewiseStep = prx::experimental::piecewise_step_t<ControlType, DurationType>;
  sampler_t(const prx::param_loader params) : _ctrl_sampler(params["control"]), _duration_sampler(params["duration"])
  {
  }

  PiecewiseStep operator()()
  {
    return std::move(PiecewiseStep(_ctrl_sampler(), _duration_sampler()));
  }

protected:
  sampler_t<ControlType> _ctrl_sampler;
  sampler_t<DurationType> _duration_sampler;
};

template <typename ControlType, typename DurationType>
class sampler_t<prx::experimental::piecewise_plan_t<ControlType, DurationType>>
{
public:
  using PiecewiseStep = prx::experimental::piecewise_step_t<ControlType, DurationType>;
  using PiecewisePlan = prx::experimental::piecewise_plan_t<ControlType, DurationType>;
  sampler_t(const prx::param_loader params) : _step_sampler(params["step"]), _total_steps_sampler(params["total_steps"])
  {
  }

  PiecewisePlan operator()()
  {
    PiecewisePlan plan{};
    const int total{ _total_steps_sampler() };
    for (int i = 0; i < total; ++i)
    {
      const PiecewiseStep step{ _step_sampler() };
      plan.push_back(step);
    }
    return std::move(plan);
  }

protected:
  sampler_t<PiecewiseStep> _step_sampler;
  sampler_t<int> _total_steps_sampler;
};

}  // namespace prx