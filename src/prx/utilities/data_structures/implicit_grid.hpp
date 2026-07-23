#pragma once

#include <bit>
#include <bitset>
#include <map>
#include <gtsam/base/Lie.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <prx/utilities/math/primes.hpp>
#include "prx/utilities/general/transforms.hpp"
#include "prx/utilities/general/debug_utils.hpp"
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
namespace prx
{
template <typename State>
class cell_size_factor_t : public gtsam::NoiseModelFactorN<Eigen::Vector<double, gtsam::traits<State>::dimension>>
{
  static constexpr Eigen::Index DimX{ gtsam::traits<State>::dimension };
  using Tangent = Eigen::Vector<double, DimX>;

  using Base = gtsam::NoiseModelFactorN<Tangent>;
  using Derived = cell_size_factor_t<State>;

  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

  cell_size_factor_t() = delete;

public:
  cell_size_factor_t(const cell_size_factor_t& other) = delete;

  cell_size_factor_t(const gtsam::Key key_x0, const double cell_size)
    : Base(nullptr, key_x0), _cell_size(cell_size * Tangent::Ones()), _xc(State())
  {
  }

  ~cell_size_factor_t() override
  {
  }

  // Error is: z (-) q_^{predicted}_1; where q_^{predicted}_1 = q0 (+) qdot dt, for a fix (known) dt
  virtual Eigen::VectorXd evaluateError(const Tangent& tg,  // no-lint
                                        OptDeriv Htg = boost::none) const override
  {
    using Jac = Eigen::Matrix<double, DimX, DimX>;
    Jac xp_H_tg, xm_H_mtg, xh_H_xp, xl_H_xm, cs_H_xB, xB_H_xl, xB_H_xp;
    // const Jac mtg_H_tg{ -Jac::Identity() };

    // const State xm{ gtsam::traits<State>::Expmap(-tg, xm_H_mtg) };
    // const State x_low{ gtsam::traits<State>::Compose(_xc, xm, boost::none, xl_H_xm) };

    const State xp{ gtsam::traits<State>::Expmap(tg, xp_H_tg) };
    // const State x_high{ gtsam::traits<State>::Compose(_xc, xp, boost::none, xh_H_xp) };
    // const State x_high_inv{ gtsam::traits<State>::Inverse(x_high, xhI_H_xh) };

    // const State xB{ gtsam::traits<State>::Between(x_low, x_high, xB_H_xl, xB_H_xh) };
    const State xB{ gtsam::traits<State>::Between(_xc, xp, boost::none, xB_H_xp) };
    const Tangent cs{ gtsam::traits<State>::Logmap(xB, cs_H_xB) };
    const Tangent error{ cs - _cell_size };

    // PRX_DBG_VARS(tg.transpose());
    // PRX_DBG_VARS(_xc);
    // PRX_DBG_VARS(xm, x_low);
    // PRX_DBG_VARS(xp, x_high);
    // PRX_DBG_VARS(xB);
    // PRX_DBG_VARS(cs.transpose(), _cell_size.transpose());
    // PRX_DBG_VARS(error.transpose());
    if (Htg)
    {
      *Htg = cs_H_xB * xB_H_xp * xp_H_tg;  // +             // High
                                           // cs_H_xB * xB_H_xl * xl_H_xm * xm_H_mtg * mtg_H_tg;  // Low
      // PRX_DBG_VARS(cs_H_xLH);
      // PRX_DBG_VARS(xLH_H_xh);
      // PRX_DBG_VARS(xh_H_xp);
      // PRX_DBG_VARS(xp_H_tg);
      // PRX_DBG_VARS(cs_H_xLH);
      // PRX_DBG_VARS(xLH_H_xl);
      // PRX_DBG_VARS(xl_H_xm);
      // PRX_DBG_VARS(xm_H_mtg);
      // PRX_DBG_VARS(mtg_H_tg);
      // PRX_DBG_VARS(*Htg);
    }
    return error;
  }

private:
  const State _xc;
  const Tangent _cell_size;
};

template <typename LieType, typename CellType>
class implicit_grid_t
{
  static constexpr Eigen::Index Dimension{ gtsam::traits<LieType>::dimension };

  // Fwd declaration
  struct state_compare_t;

public:
  using Vertex = Eigen::Vector<int, Dimension>;
  using TangentElement = Eigen::Vector<double, Dimension>;
  using CellsMap = std::map<Vertex, CellType, state_compare_t>;
  using Iterator = typename CellsMap::iterator;
  using ConstIterator = typename CellsMap::const_iterator;

  implicit_grid_t() : _x0(LieType()), _cell_sizes(TangentElement::Ones()), _hashing_vector(init_with_primes<Vertex>())
  {
    for (int i = 0; i < Dimension; ++i)
    {
      _pos_vector[i] = 1 << i;
    }
    PRX_DBG_VARS(_hashing_vector, _pos_vector);
  }

  LieType x0() const
  {
    return _x0;
  }
  void reset(const LieType x0, const double cell_size)
  {
    gtsam::Values values;
    gtsam::NonlinearFactorGraph graph;
    const gtsam::Key k{ gtsam::Symbol('x', 0) };

    const TangentElement init_val{ cell_size * TangentElement::Ones() };
    values.insert(k, init_val);
    graph.emplace_shared<cell_size_factor_t<LieType>>(k, cell_size);
    values = gtsam::GaussNewtonOptimizer(graph, values).optimize();
    // values.print();
    const TangentElement cell_sizes{ values.at<TangentElement>(k) };
    reset(x0, cell_sizes);
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
  Vertex vertex(const LieType& xi) const
  {
    const LieType x0i{ gtsam::traits<LieType>::Compose(_x0_inv, xi) };
    const TangentElement eps{ gtsam::traits<LieType>::Logmap(x0i) * 1.0001 };
    const TangentElement eps_div{ eps.cwiseQuotient(_cell_sizes) };
    const TangentElement v_grid_dbl{ eps_div.unaryExpr(&implicit_grid_t::unary_modf) };
    const Vertex v_grid{ v_grid_dbl.template cast<int>() };

    return v_grid;
  }

  // std::bit_cast is available in c++20. Currently, this library targets c++17...
  // so here is an implementation (from cppreference.com)
  template <class To, class From>
  std::enable_if_t<sizeof(To) == sizeof(From) && std::is_trivially_copyable_v<From> && std::is_trivially_copyable_v<To>,
                   To> static bit_cast(const From& src) noexcept
  {
    static_assert(std::is_trivially_constructible_v<To>,
                  "This implementation additionally requires "
                  "destination type to be trivially constructible");

    To dst;
    std::memcpy(&dst, &src, sizeof(To));
    return dst;
  }
  // Fast hash from: T. Matthias, et al. "Optimized spatial hashing for collision detection of deformable objects."
  // This hash seeks to differentiate between representative vertices, so there is an implicit assumption that the
  // hashing elements are "far-enough" between each other.
  // From testing: TangentElement::Ones() * 0.1 gets the same hash as Tangent::Ones() * 0.10000001
  // Which is well enough... Possible improvements: compute two or more hashes and check or use bigger primes
  std::size_t hash(const Vertex& vx) const
  {
    const Vertex aux{ vx.cwiseProduct(_hashing_vector) };
    const Vertex aux2{ aux.cwiseProduct(_pos_vector) };

    // <<<<<<< HEAD
    const std::size_t h{ static_cast<std::size_t>(aux2.redux(&implicit_grid_t::xor_reductor)) };
    return h;
    // return static_cast<std::size_t>(h_dbl);
    // =======
    //     const double h_dbl{ aux.redux(&implicit_grid_t::xor_reductor) };
    //     const std::size_t h{ bit_cast<std::size_t>(h_dbl) };
    //     // PRX_DBG_VARS(tg, aux, h_dbl, h);
    //     return h;
    // >>>>>>> 1ea071c5 (adding bitcast fn for c++17)
  }

  template <typename Lie, std::enable_if_t<not std::is_same_v<Lie, TangentElement>, bool> = true>
  std::size_t hash(const Lie& x) const
  {
    const Vertex tg{ vertex(x) };
    return hash(tg);
  }
  std::size_t hash(const TangentElement& x) const
  {
    const LieType x_local{ gtsam::traits<LieType>::Expmap(x) };
    const Vertex tg{ vertex(x_local) };
    return hash(tg);
  }

  bool exists(const LieType& x)
  {
    const Vertex v{ vertex(x) };
    return _cells.count(v) > 0;
  }

  // Return the state associated to the Local tangent element.
  // As the tangent element us local with respect to x0, this function does:
  // res = x0 * Expmap(tg)
  LieType state(const TangentElement& vx)
  {
    const LieType x_local{ gtsam::traits<LieType>::Expmap(vx) };
    const LieType x_global{ gtsam::traits<LieType>::Compose(_x0, x_local) };
    return x_global;
  }

  LieType state_from_vertex(const Vertex& vx) const
  {
    const TangentElement vx_am{ vx.template cast<double>() };
    const TangentElement vx_prod{ vx_am.cwiseProduct(_cell_sizes) };
    const LieType x_local{ gtsam::traits<LieType>::Expmap(vx_prod) };
    const LieType x_global{ gtsam::traits<LieType>::Compose(_x0, x_local) };

    return x_global;
  }

  TangentElement center(const LieType& xi) const
  {
    const Vertex v{ vertex(xi) };
    const TangentElement vp{ v.template cast<double>().cwiseProduct(_cell_sizes) };
    // const LieType x0i{ gtsam::traits<LieType>::Compose(_x0_inv, xi) };
    // const TangentElement v{ gtsam::traits<LieType>::Logmap(x0i) };
    const TangentElement c{ vp + _cell_sizes / 2.0 };
    return std::move(c);
  }

  std::vector<Vertex> vertices(const LieType& xi) const
  {
    const std::size_t total_vertices{ static_cast<std::size_t>(std::pow(2, Dimension)) };

    std::vector<Vertex> vxs;  // total_vertices, vertex(xi));
    vxs.push_back(vertex(xi));

    for (int i = 1; i < total_vertices; ++i)
    {
      _visitor._bits = i;
      vxs.front().visit(_visitor);
      vxs.push_back(_visitor.result);
    }
    return vxs;
  }

  void set_cell(const LieType& xi, CellType value)
  {
    const Vertex v{ vertex(xi) };
    // _cells[v] = value;
    // PRX_DBG_VARS(v.transpose());

    auto [iter, flag] = _cells.insert_or_assign(v, value);
    // PRX_DBG_VARS(iter->first, iter->second, flag)
  }

  template <typename Lie, std::enable_if_t<not std::is_same_v<Lie, TangentElement>, bool> = true>
  CellType& cell(const Lie& xi)
  {
    const Vertex v_cell{ vertex(xi) };
    // PRX_DBG_VARS(size(), v_cell.transpose());
    return _cells[v_cell];
  }

  CellType& cell(const Vertex& vx)
  {
    return _cells[vx];
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
    const double res{ std::modf(x + 0.0001, &ptr) };
    // PRX_DBG_VARS(x, res, ptr)
    return x < 0. ? ptr - 1. : ptr;
  };

  static double unary_antimodf(const double& x)
  {
    // return x < 0. ? x + 1. : x;
    return x;
  }

  static int xor_reductor(const int& x, const int& y)
  {
    const int res{ (x << 2) ^ y };
    return res;
  }
  struct state_compare_t
  {
    bool operator()(const Vertex& lhs, const Vertex& rhs) const
    {
      return std::lexicographical_compare(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
      // for (int i = 0; i < Dimension; ++i)
      // {
      //   // PRX_DBG_VARS(i, lhs[i], rhs[i])
      //   if (lhs[i] < rhs[i])
      //   {
      //     return true;
      //   }
      // }
      // return false;
    }
  };

  struct vertices_visitor_t
  {
    TangentElement cell_sizes;
    Vertex result;
    std::bitset<Dimension> _bits;
    // _bits.reset();
    // called for the first coefficient
    void init(const double& value, Eigen::Index i, Eigen::Index j)
    {
      this->operator()(value, i, j);
    }
    // called for all other coefficients
    void operator()(const int& value, Eigen::Index i, Eigen::Index j)
    {
      result[i] = value;
      if (_bits[i])
      {
        result[i] += 1;  // cell_sizes[i];
      }
    }
  };

  mutable vertices_visitor_t _visitor;
  TangentElement _cell_sizes;
  Vertex _hashing_vector, _pos_vector;
  LieType _x0, _x0_inv;
  CellsMap _cells;
};
}  // namespace prx