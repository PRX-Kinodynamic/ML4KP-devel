#pragma once

#include <memory>
#include <fstream>
#include <string>
#include <unordered_map>

#include <gtsam/inference/Key.h>

#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/hash.hpp"
namespace prx
{
namespace fg
{

using symbol_t = gtsam::Key;

class symbol_factory_t
{
public:
  template <typename... Ts>
  static gtsam::Key create_hashed_symbol(std::string name, Ts... args)
  {
    std::size_t hash{ 0 };

    const std::tuple<std::string, Ts...> ts(name, args...);
    const std::string str{ create_hash<0>(hash, ts) };
    const gtsam::Key key{ gtsam::Key(hash) };

    symbols_map[key] = str;
    return key;
  }

  static void symbols_to_file(std::string filename = prx::out_path + "factor_graphs_symbols.txt")
  {
    std::ofstream ofs;
    ofs.open(filename.c_str(), std::ofstream::trunc);

    for (auto key_str : symbols_map)
    {
      ofs << key_str.second << " " << key_str.first << " " << gtsam::DefaultKeyFormatter(key_str.first) << "\n";
    }

    ofs.close();
  }

  static std::string formatter(const gtsam::Key key)
  {
    return symbols_map[key];
  }

  template <std::size_t I, typename... Tp, std::enable_if_t<(I == sizeof...(Tp) - 1), bool> = true>
  inline static std::string create_hash(std::size_t& hash, const std::tuple<Tp...>& t)
  {
    std::stringstream str;
    str << std::get<I>(t);
    return str.str();
  }
  template <std::size_t I, typename... Tp, std::enable_if_t<(I < sizeof...(Tp) - 1), bool> = true>
  inline static std::string create_hash(std::size_t& hash, const std::tuple<Tp...>& t)
  {
    std::stringstream str;
    prx::hash_combine(hash, std::get<I>(t));
    str << std::get<I>(t);
    str << create_hash<I + 1>(hash, t);

    hash = std::hash<std::string>()(str.str());
    return str.str();
  }

private:
  static inline std::unordered_map<gtsam::Key, std::string> symbols_map;

  symbol_factory_t() {};
};

}  // namespace fg
}  // namespace prx

#define PRINT_KEYS(...)                                                                                                \
  {                                                                                                                    \
    const std::vector<gtsam::Key> _keys{ __VA_ARGS__ };                                                                \
    std::vector<std::string> keys;                                                                                     \
    for (auto _key : _keys)                                                                                            \
    {                                                                                                                  \
      keys.push_back(SF::formatter(_key));                                                                             \
    }                                                                                                                  \
    PRX_DBG_VARS(keys)                                                                                                 \
  };