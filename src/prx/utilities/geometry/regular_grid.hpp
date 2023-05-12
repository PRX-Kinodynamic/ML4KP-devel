#pragma once

#include "prx/utilities/defs.hpp"

#include <type_traits>
#include <unordered_map>

namespace prx
{

template <typename value_t, std::size_t dimension>
class regular_grid_t
{
public:
  const std::size_t Dim = dimension;
  using key_t = std::array<int, dimension>;
  using hash_function_t = range_hash_combine_t<std::array<int, dimension>, dimension>;
  using const_iterator = typename std::unordered_map<key_t, value_t, hash_function_t>::const_iterator;

  regular_grid_t() = delete;
  /**
   * @brief      Constructs a new instance of grid from bounds and size of grid
   *
   * @param[in]  bounds  The bounds of the grid on each dimension: (lower[i], upper[i]) (i.e. space_t::get_bounds())
   * @param[in]  divisions  Number of divisions in each dimension. Either a number (same value for all dimensions) or
   * DIM numbers
   *
   * @tparam     Ts      Arithmetic types. Either one value or dimension total values
   */
  template <class... Ts, std::enable_if_t<(sizeof...(Ts) == 1), bool> = true>
  regular_grid_t(const std::vector<std::pair<double, double>> bounds, Ts... divisions) : _bounds(bounds)
  {
    prx_assert(bounds.size() == dimension, "Dimension must be: " << dimension << "; got: " << bounds.size());
    std::array<int, dimension> size{};

    std::tuple<double> tuple{ divisions... };
    std::fill(size.begin(), size.end(), std::get<0>(tuple));
    compute_lambda(size, bounds);
  }

  template <class... Ts, std::enable_if_t<(sizeof...(Ts) > 1 && sizeof...(Ts) == dimension), bool> = true>
  regular_grid_t(const std::vector<std::pair<double, double>> bounds, Ts... divisions) : _bounds(bounds)
  {
    prx_assert(bounds.size() == dimension, "Dimension must be: " << dimension << "; got: " << bounds.size());
    std::array<int, dimension> size{ divisions... };
    compute_lambda(size, bounds);
  }

  regular_grid_t(const regular_grid_t<value_t, dimension>& other) : _bounds(other._bounds)
  {
    for (auto cell : other)
    {
      memory[cell.first] = cell.second;
    }
    lambda = other.lambda;
  }

  void populate_grid(const value_t& initial_value)
  {
    using Container = std::vector<double>;
    std::function<value_t(const Container&)> initializer = [&initial_value](const Container&) { return initial_value; };
    populate_grid(initializer);
  }

  template <class Container>
  void populate_grid(std::function<value_t(const Container&)>& initializer)
  {
    // def state_increment(space_point, step_inc, lower_bounds, upper_bounds):
    std::array<double, dimension> pt;
    std::array<double, dimension> lower_bounds;
    std::array<double, dimension> upper_bounds;
    std::array<double, dimension> steps;
    for (int i = 0; i < dimension; ++i)
    {
      pt[i] = _bounds[i].first;
      lower_bounds[i] = _bounds[i].first;
      upper_bounds[i] = _bounds[i].second;
      steps[i] = get_cell_length(i);
    }
    int i = 0;
    do
    {
      memory[mapping(pt)] = initializer(Container{ pt.begin(), pt.end() });

      // for (i = 0; i < dimension; ++i)
      // {
      //   const double step_inc{ get_cell_length(i) };
      //   pt[i] = pt[i] + step_inc;

      //   if (pt[i] <= upper_bounds[i])
      //   {
      //     break;
      //   }
      //   pt[i] = lower_bounds[i];
      // }
    } while (state_space_step(pt, steps, dimension, lower_bounds, upper_bounds));
  }

  const_iterator begin() const noexcept
  {
    return memory.begin();
  }

  const_iterator end() const noexcept
  {
    return memory.end();
  }

  /**
   * @brief      Array indexer operator for a given key with no mapping needed to the key. Intended to use if the key
   * comes from another grid. For accessing a cell with unknown key, use parenthesis --regular_grid::operator()--
   * instead
   *
   * @param[in]  key   The key to use
   *
   * @return     The value_t associated with the key
   */
  value_t operator[](const key_t key) const
  {
    return memory[key];
  }

  value_t& operator[](const key_t key)
  {
    return memory[key];
  }

  /**
   * @brief      Function call operator. Values_in are used to generate a key associated to a value.
   *
   * @param[in]  values_in  The values associated to the stored element. As many arguments as the dimension of the
   * grid (as a grid would imply).
   *
   * @tparam     Ts         Types - as many doubles as dimension of the grid
   *
   * @return     The value_t associated with the cell of this grid.
   */
  template <class... Ts>
  const value_t operator()(const Ts... values_in) const
  {
    static_assert(sizeof...(values_in) == dimension, "[Dimension mismatch] regular_grid_t::operator().");
    std::array<double, dimension> values{ values_in... };
    const key_t key{ mapping(values) };
    if (memory.find(key) == memory.end())
    {
      PRX_DEBUG_ITERABLE(values);
      prx_throw("regular_grid_t - key not found!");
    }
    return memory.at(key);
  }

  template <class... Ts>
  value_t& operator()(const Ts... values_in)
  {
    static_assert(sizeof...(values_in) == dimension, "[Dimension mismatch] regular_grid_t::operator().");
    std::array<double, dimension> values{ values_in... };
    return memory[mapping(values)];
  }

  // WIP - Implementation not finished... what is the best way to pass which ones to ignore?
  // template <int ignore, class... Ts>
  // std::vector<std::pair<key_t, value_t>> operator()(const Ts... values_in)
  // {
  //   static_assert(sizeof...(values_in) == dimension, "[Dimension mismatch] regular_grid_t::operator().");
  //   std::array<double, dimension> values{ values_in... };

  //   std::vector<std::pair<key_t, value_t>> result{};
  //   for (auto cell : memory)
  //   {
  //     const key_t key{ mapping(values) };
  //     bool match{ true };
  //     for (int i = 0; i < dimension; ++i)
  //     {
  //       if (cell.first[i] != key[i] && values[i] != ignore)
  //       {
  //         i = dimension;  // early exit
  //         match = false;
  //       }
  //     }
  //     if (match)
  //     {
  //       result.push_back(std::make_pair(key, cell.second));
  //     }
  //   }
  //   return result;
  // }

  void set_to(const value_t rhs)  // compound assignment (does not need to be a
                                  // member,
  {                               // but often is, to modify the private members)
    for (auto cell : memory)
    {
      memory[cell.first] = rhs;
    }
  }

  regular_grid_t<value_t, dimension>& operator+=(const value_t rhs)  // compound assignment (does not need to be a
                                                                     // member,
  {                                                                  // but often is, to modify the private members)
    for (auto cell : memory)
    {
      memory[cell.first] += rhs;
    }
    return *this;  // return the result by reference
  }

  template <typename value_other_t>
  friend regular_grid_t<value_t, dimension> operator+(const regular_grid_t<value_t, dimension>& lhs,
                                                      const regular_grid_t<value_other_t, dimension>& rhs)
  {
    regular_grid_t<value_t, dimension> result(lhs);
    for (auto cell : rhs)
    {
      result.memory[cell.first] = lhs.memory[cell.first] + cell.second;
    }
    return result;
  }

  template <typename value_other_t>
  friend regular_grid_t<value_t, dimension> operator-(const regular_grid_t<value_t, dimension>& lhs,
                                                      const regular_grid_t<value_other_t, dimension>& rhs)
  {
    regular_grid_t<value_t, dimension> result(lhs);
    for (auto cell : rhs)
    {
      result.memory[cell.first] = lhs.memory[cell.first] - cell.second;
    }
    return result;
  }

  friend std::ostream& operator<<(std::ostream& os, const regular_grid_t<value_t, dimension>& obj)
  {
    for (auto cell : obj.memory)
    {
      os << "[";
      for (int i = 0; i < dimension; ++i)
      {
        os << cell.first[i];
        if (i < dimension - 1)
          os << ", ";
      }
      os << "]: " << cell.second << "\n";
    }
    return os;
  }

  void to_file(const std::string filename, const std::ios_base::openmode _mode = std::ofstream::trunc) const
  {
    auto passthrough_formatter = [](const value_t& v) { return v; };
    to_file(filename, passthrough_formatter, _mode);
  }

  template <typename FormattingFunction>
  void to_file(const std::string filename, FormattingFunction formatter,
               const std::ios_base::openmode _mode = std::ofstream::trunc) const
  {
    std::ofstream ofs;
    ofs.open(filename.c_str(), _mode);
    const std::size_t one{ 1 };

    for (auto cell : memory)
    {
      std::array<double, dimension> uk = unmap_key<std::array<double, dimension>>(cell.first);
      for (std::size_t j = 0; j < dimension; ++j)
      {
        // const std::size_t mask{ one << j };
        // const std::size_t masked{ i & mask };
        // const std::size_t val{ masked >> j };
        ofs << uk[j];
        ofs << prx::separating_value;
      }
      for (std::size_t j = 0; j < dimension; ++j)
      {
        ofs << get_cell_length(j);
        ofs << prx::separating_value;
      }
      ofs << formatter(cell.second) << "\n";
    }
    ofs << std::endl;
  }

  inline std::size_t size() const
  {
    return memory.size();
  }

  // Get "real" coordinates corresponding to the map. Note that since multiple coordinates are mapped to the same key,
  // we cannot retrive the "original" value passed to the grid, only \textit{a} value associated to the key.
  template <typename Ret>
  const Ret unmap_key(const key_t& key) const
  {
    Ret raw_val{};
    for (int i = 0; i < dimension; ++i)
    {
      raw_val[i] = key[i] / lambda[i];
    }
    return raw_val;
  }

  template <typename Ret, class... Ts>
  const Ret unmap(const Ts... values_in) const
  {
    std::array<double, dimension> values{ values_in... };

    return unmap_key<Ret>(mapping(values));
  }

  inline double get_cell_length(const std::size_t& dim_at) const
  {
    return 1.0 / lambda[dim_at];
  }

  inline std::pair<double, double> bounds(const std::size_t i) const
  {
    return _bounds[i];
  }

  // ToDO: make this varadic
  template <class... Ts>
  inline bool in_bounds(const Ts... values_in) const
  {
    bool ans = true;
    // value_t& operator()(const Ts... values_in)
    // {
    static_assert(sizeof...(values_in) == dimension, "[Dimension mismatch] regular_grid_t::operator().");
    std::array<double, dimension> values{ values_in... };
    for (int i = 0; i < dimension; ++i)
    {
      ans &= _bounds[i].first <= values[i];
      ans &= values[i] <= _bounds[i].second;
    }
    return ans;
  }

private:
  key_t mapping(std::array<double, dimension> raw_key) const
  {
    std::array<int, dimension> key{};
    for (int i = 0; i < dimension; ++i)
    {
      key[i] = std::floor(lambda[i] * std::max(std::min(raw_key[i], _bounds[i].second), _bounds[i].first));
    }
    return key;
  }

  void compute_lambda(std::array<int, dimension> grid_size, std::vector<std::pair<double, double>> bounds)
  {
    for (int i = 0; i < dimension; ++i)
    {
      lambda[i] = grid_size[i] / std::fabs(bounds[i].second - bounds[i].first);
    }
  }

  const std::vector<std::pair<double, double>> _bounds;
  std::unordered_map<key_t, value_t, hash_function_t> memory;

  std::array<double, dimension> lambda{};
};
}  // namespace prx