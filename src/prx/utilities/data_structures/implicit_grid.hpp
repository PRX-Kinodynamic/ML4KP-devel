#pragma once

#include <bitset>
#include <map>
#include <gtsam/base/Lie.h>
#include <prx/utilities/math/primes.hpp>
#include "prx/utilities/general/transforms.hpp"

namespace prx
{
template <typename LieType, typename CellType>
class implicit_grid_t
{
  static constexpr Eigen::Index Dimension{ gtsam::traits<LieType>::dimension };

  // Fwd declaration
  struct state_compare_t;

public:
  using TangentElement = Eigen::Vector<double, Dimension>;
  using CellsMap = std::map<TangentElement, CellType, state_compare_t>;
  using Iterator = typename CellsMap::iterator;
  using ConstIterator = typename CellsMap::const_iterator;

  implicit_grid_t()
    : _x0(LieType()), _cell_sizes(TangentElement::Ones()), _hashing_vector(init_with_primes<TangentElement>())
  {
  }

  LieType x0() const
  {
    return _x0;
  }

  void reset(const LieType x0, const TangentElement cell_sizes)
  {
    _x0 = x0;
    _cell_sizes = cell_sizes;
    _visitor.cell_sizes = _cell_sizes;
    _x0_inv = gtsam::traits<LieType>::Inverse(x0);
    _cells.clear();
  }

  void clear()
  {
    _cells.clear();
  }

  // Representative vertex of the cube where xi lies
  TangentElement vertex(const LieType& xi) const
  {
    const LieType x0i{ gtsam::traits<LieType>::Compose(_x0_inv, xi) };
    const TangentElement eps{ gtsam::traits<LieType>::Logmap(x0i) };
    const TangentElement eps_div{ eps.cwiseQuotient(_cell_sizes) };
    const TangentElement v_grid{ eps_div.unaryExpr(&implicit_grid_t::unary_modf) };
    return v_grid;
  }

  // Fast hash from: T. Matthias, et al. "Optimized spatial hashing for collision detection of deformable objects."
  // This hash seeks to differentiate between representative vertices, so there is an implicit assumption that the
  // hashing elements are "far-enough" between each other.
  // From testing: TangentElement::Ones() * 0.1 gets the same hash as Tangent::Ones() * 0.10000001
  // Which is well enough... Possible improvements: compute two or more hashes and check or use bigger primes
  std::size_t hash(const TangentElement& tg) const
  {
    const TangentElement aux{ tg.cwiseProduct(_hashing_vector) };

    const double h_dbl{ aux.redux(&implicit_grid_t::xor_reductor) };
    return static_cast<std::size_t>(h_dbl);
  }

  template <typename Lie, std::enable_if_t<not std::is_same_v<Lie, TangentElement>, bool> = true>
  std::size_t hash(const Lie& x) const
  {
    const TangentElement tg{ vertex(x) };
    return hash(tg);
  }

  // Return the state associated to the Local tangent element.
  // As the tangent element us local with respect to x0, this function does:
  // res = x0 * Expmap(tg)
  LieType state(const TangentElement& tg)
  {
    const LieType x_local{ gtsam::traits<LieType>::Expmap(tg) };
    const LieType x_global{ gtsam::traits<LieType>::Compose(_x0, x_local) };
    return x_global;
  }

  TangentElement center(const LieType& xi) const
  {
    const TangentElement v{ vertex(xi) };
    const TangentElement vp{ v.cwiseProduct(_cell_sizes) };
    // const LieType x0i{ gtsam::traits<LieType>::Compose(_x0_inv, xi) };
    // const TangentElement v{ gtsam::traits<LieType>::Logmap(x0i) };
    const TangentElement c{ vp + _cell_sizes / 2.0 };
    return std::move(c);
  }

  std::vector<TangentElement> vertices(const LieType& xi)
  {
    const std::size_t total_vertices{ static_cast<std::size_t>(std::pow(2, Dimension)) };

    std::vector<TangentElement> vxs;  // total_vertices, vertex(xi));
    vxs.push_back(vertex(xi));

    for (int i = 1; i < total_vertices; ++i)
    {
      _visitor._bits = i;
      vxs.front().visit(_visitor);
      vxs.push_back(_visitor.result);
    }
    return vxs;
  }

  CellType& cell(const LieType& xi)
  {
    const TangentElement v{ vertex(xi) };
    return _cells[v];
  }

  TangentElement cell_sizes() const
  {
    return _cell_sizes;
  }

  ConstIterator begin()
  {
    return _cells.begin();
  }

  ConstIterator end()
  {
    return _cells.end();
  }

  std::size_t size()
  {
    return _cells.size();
  }

private:
  // std::function<double(double)> uniform_sample = [&](const double x) { return dist(gen); };
  static double unary_modf(const double& x)
  {
    double ptr;
    std::modf(x, &ptr);
    return x < 0. ? ptr - 1. : ptr;
  };

  static double xor_reductor(const double& x, const double& y)
  {
    return static_cast<double>(std::lrint(x) ^ std::lrint(y));
  }
  struct state_compare_t
  {
    bool operator()(const TangentElement& lhs, const TangentElement& rhs) const
    {
      for (int i = 0; i < Dimension; ++i)
      {
        if (lhs[i] < rhs[i])
        {
          return true;
        }
      }
      return false;
    }
  };

  struct vertices_visitor_t
  {
    TangentElement cell_sizes;
    TangentElement result;
    std::bitset<Dimension> _bits;
    // _bits.reset();
    // called for the first coefficient
    void init(const double& value, Eigen::Index i, Eigen::Index j)
    {
      this->operator()(value, i, j);
    }
    // called for all other coefficients
    void operator()(const double& value, Eigen::Index i, Eigen::Index j)
    {
      result[i] = value;
      if (_bits[i])
      {
        result[i] += cell_sizes[i];
      }
    }
  };

  vertices_visitor_t _visitor;
  TangentElement _cell_sizes;
  TangentElement _hashing_vector;
  LieType _x0, _x0_inv;
  CellsMap _cells;
};
}  // namespace prx