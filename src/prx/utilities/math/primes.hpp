#pragma once

#include <array>

namespace prx
{
// Using: https://www.bigprimes.net/
constexpr std::array<int, 9> primes = { 1022387,   // no-lint
                                        10000019,  // no-lint
                                        22231789,  // no-lint
                                        72546283,  // no-lint
                                        99999989,  // no-lint
                                        15490663,  // no-lint
                                        29054941,  // no-lint
                                        46491799,  // no-lint
                                        82427857 };

template <typename Container>
Container init_with_primes(const std::size_t total_primes)
{
  Container c(total_primes);
  for (int i = 0; i < total_primes; ++i)
  {
    c[i] = prx::primes[i % prx::primes.size()];
  }
  return c;
}

// Init a given container with N prime numbers. If N > prx::primes, primes start repeating.
template <typename Container>
Container init_with_primes()
{
  return init_with_primes<Container>(Container().size());
}

}  // namespace prx