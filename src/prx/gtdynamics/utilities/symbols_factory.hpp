#pragma once

#include <memory>
#include <unordered_map>
#include <string>

#include "prx/simulation/system.hpp"
#include "prx/gtdynamics/utilities/prx_symbols.hpp"

namespace prx
{

// class prx_symbol_t;

typedef std::function<prx_symbol_t(std::string, uint8_t state_idx, uint64_t t)> symbol_gen_fn;

class symbol_factory_t
{
public:
  static symbol_factory_t& get()
  {
    static symbol_factory_t instance;
    return instance;
  }

  /**
   * When called, this methods creates an instance of the system associated to the given name and assigns the given
   * path.
   * @param  name Name of the system to create
   * @param  path Path associated with the system
   * @return      The instance of the system or nullptr if the system is not registred.
   */
  template <typename... Ts>
  static prx_symbol_t create_symbol(std::string name, Ts... args)
  {
    auto it = symbol_factory_t::get().symbol_generators.find(name);
    if (it != symbol_factory_t::get().symbol_generators.end())
    {
      // Hacky thing to guarantee size > 2 at compilation
      std::tuple<Ts..., int, int> tp(args..., 0, 0);
      const int size = sizeof...(args);
      switch (size)
      {
        case 0:
          return it->second("", 0, 0);
          break;
        case 1:
          return it->second("", 0, std::get<0>(tp));
          break;
        case 2:
          return it->second("", std::get<0>(tp), std::get<1>(tp));
          break;
        default:
          prx_throw("Could not create symbol: " << name << ". Got " << size << " arguments and maximum allowed is 2");
      }
    }

    prx_throw("Could not create symbol: " << name);
    return prx_symbol_t();
  }

  bool register_symbol(const std::string name, symbol_gen_fn func)
  {
    bool res;
    auto it = symbol_factory_t::get().symbol_generators.find(name);
    if (it != symbol_factory_t::get().symbol_generators.end())
    {
      res = true;
    }
    else
    {
      res = symbol_generators.insert(std::make_pair(name, func)).second;
      prx_assert(res, "Problem registering system: " << name);
    }
    return res;
  }

  /**
   * Returns the names of the available systems
   */
  static std::vector<std::string> available_symbols()
  {
    std::vector<std::string> r;
    for (auto it : symbol_factory_t::get().symbol_generators)
    {
      r.push_back(it.first);
    }
    return r;
  }

private:
  std::unordered_map<std::string, symbol_gen_fn> symbol_generators;

  symbol_factory_t(){};
};

}  // namespace prx

/**
 * @brief      Macro to register a symbol
 *
 * @param      SYMBOL_NAME  Name to use inside the factory
 * @param      SYMBOL_STR   Maximum of two-char string to use as variable identifier. If an empty string is used, the
 * symbol is constructed from the corresponding parameter suplied to symbol_factory_t::create_symbol
 * @param      SYMBOL_IDX   Symbol numeric identifier. If a 0 is used, the symbol is constructed from the corresponding
 * parameter suplied to symbol_factory_t::create_symbol
 * @param      SYMBOL_T     The time symbol. . If a 0 is used, the symbol is constructed from the corresponding
 * parameter suplied to symbol_factory_t::create_symbol
 *
 * @return     { description_of_the_return_value }
 */
#define PRX_REGISTER_SYMBOL(SYMBOL_NAME, SYMBOL_STR, SYMBOL_IDX, SYMBOL_T)                                             \
  namespace prx                                                                                                        \
  {                                                                                                                    \
  namespace factory_registration                                                                                       \
  {                                                                                                                    \
  static auto FN_##SYMBOL_NAME##_GENERATOR = [](const std::string& _s, int8_t _state_idx, int64_t _t) {                \
    const std::string str = _s == "" ? SYMBOL_STR : _s;                                                                \
    const int8_t idx = _state_idx == 0 ? SYMBOL_IDX : _state_idx;                                                      \
    const int64_t t = _t == 0 ? SYMBOL_T : _t;                                                                         \
    return prx_symbol_t(str, idx, t);                                                                                  \
  };                                                                                                                   \
  const bool VAR_##SYMBOL_NAME##_REGISTRED =                                                                           \
      symbol_factory_t::get().register_symbol(#SYMBOL_NAME, FN_##SYMBOL_NAME##_GENERATOR);                             \
  }                                                                                                                    \
  }
