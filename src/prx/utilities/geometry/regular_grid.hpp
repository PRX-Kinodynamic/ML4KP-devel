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
  using key_t = std::array<int, dimension>;
  using hash_function_t = range_hash_combine_t<std::array<int, dimension>, dimension>;
  using const_iterator = typename std::unordered_map<key_t, value_t, hash_function_t>::const_iterator;

  regular_grid_t()
  {
    std::fill(lambda.begin(), lambda.end(), 1);
  }
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
  regular_grid_t(const std::vector<std::pair<double, double>> bounds, Ts... divisions)
  {
    prx_assert(bounds.size() == dimension, "Dimension must be: " << dimension << "; got: " << bounds.size());
    std::array<int, dimension> size{};

    std::tuple<double> tuple{ divisions... };
    std::fill(size.begin(), size.end(), std::get<0>(tuple));
    compute_lambda(size, bounds);
  }

  template <class... Ts, std::enable_if_t<(sizeof...(Ts) > 1 && sizeof...(Ts) == dimension), bool> = true>
  regular_grid_t(const std::vector<std::pair<double, double>> bounds, Ts... divisions)
  {
    prx_assert(bounds.size() == dimension, "Dimension must be: " << dimension << "; got: " << bounds.size());
    std::array<int, dimension> size{ divisions... };
    compute_lambda(size, bounds);
  }

  regular_grid_t(const regular_grid_t<value_t, dimension>& other)
  {
    for (auto cell : other)
    {
      memory[cell.first] = cell.second;
    }
    lambda = other.lambda;
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
   * @param[in]  values_in  The values associated to the stored element. As many arguments as the dimension of the grid
   * (as a grid would imply).
   *
   * @tparam     Ts         Types - as many doubles as dimension of the grid
   *
   * @return     The value_t associated with the cell of this grid.
   */
  template <class... Ts>
  value_t operator()(Ts... values_in) const
  {
    static_assert(sizeof...(values_in) == dimension, "[Dimension mismatch] regular_grid_t::operator().");
    std::array<double, dimension> values{ values_in... };

    return memory[mapping(values)];
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

  inline std::size_t size() const
  {
    return memory.size();
  }

  // Get "real" coordinates corresponding to the map. Note that since multiple coordinates are mapped to the same key,
  // we cannot retrive the "original" value passed to the grid, only \textit{a} value associated to the key.
  std::array<double, dimension> unmap(const key_t& key)
  {
    std::array<double, dimension> raw_val{};
    for (int i = 0; i < dimension; ++i)
    {
      raw_val[i] = key[i] / lambda[i];
    }
    return raw_val;
  }

private:
  key_t mapping(std::array<double, dimension> raw_key)
  {
    std::array<int, dimension> key{};
    for (int i = 0; i < dimension; ++i)
    {
      key[i] = lambda[i] * raw_key[i];
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

  std::unordered_map<key_t, value_t, hash_function_t> memory;

  std::array<double, dimension> lambda{};
};
}  // namespace prx