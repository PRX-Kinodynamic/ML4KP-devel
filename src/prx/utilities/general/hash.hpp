#pragma once

#include <algorithm>

namespace prx
{
template <typename T>
inline void hash_combine(std::size_t& seed, const T& val)
{
  seed ^= std::hash<T>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

template <typename T, std::size_t n_i, std::enable_if_t<(n_i == 0), bool> = true>
inline void range_hash_combine(std::size_t& seed, const T& range)
{
  hash_combine(seed, range[n_i]);
}

template <typename T, std::size_t n_i, std::enable_if_t<(n_i != 0), bool> = true>
inline void range_hash_combine(std::size_t& seed, const T& range)
{
  hash_combine(seed, range[n_i]);
  range_hash_combine<T, n_i - 1>(seed, range);
}

}  // namespace prx