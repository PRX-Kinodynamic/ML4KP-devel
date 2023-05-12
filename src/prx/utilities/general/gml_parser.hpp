#pragma once
#include <string>
#include "prx/utilities/general/constants.hpp"

namespace prx
{
namespace utilities
{

struct gml_list_t;  // fwd declaration

struct gml_key_t
{
  gml_key_t() = default;
  gml_key_t(const gml_key_t& key) = default;
  gml_key_t(std::string key)
  {
    std::smatch regex_match;
    std::regex key_regex(regex_str);
    std::cmatch cm;
    const bool match{ std::regex_match(key.c_str(), cm, key_regex) };
    prx_assert(match && cm[0] == key,
               "Invalid key: " << key << " must match regex \"" << std::string(regex_str) << "\".");
    _key = key;
  }
  operator std::string() const
  {
    return _key;
  }

  friend std::ostream& operator<<(std::ostream& os, const gml_key_t& obj)
  {
    os << obj._key;
    return os;
  }

private:
  std::string _key;
  // static constexpr std::string_view regex_str = "[A-Za-z][A-Za-z0-9]*";
  static constexpr const char* regex_str = "[A-Za-z][A-Za-z0-9]*";
};

struct gml_value_t
{
  gml_value_t() = default;
  gml_value_t(const gml_value_t&) = default;
  virtual operator std::string() const = 0;
  friend std::ostream& operator<<(std::ostream& os, const gml_value_t& obj)
  {
    os << std::string(obj);
    return os;
  }
};
template <typename T>
struct gml_generic_value_t : public gml_value_t
{
  gml_generic_value_t(const T& val) : gml_value_t(), _value(val)
  {
  }
  gml_generic_value_t(const gml_generic_value_t<T>&) = default;

  template <typename Ti = T, std::enable_if_t<std::is_same<Ti, std::string>::value, bool> = true>
  std::string to_string() const
  {
    return "\"" + _value + "\"";
  }

  template <typename Ti = T, std::enable_if_t<std::is_same<Ti, gml_list_t>::value, bool> = true>
  std::string to_string() const
  {
    std::stringstream ss;
    ss << " [\n";
    ss << _value;
    ss << "]\n";
    return ss.str();
  }

  template <
      typename Ti = T,
      std::enable_if_t<!std::is_same<Ti, std::string>::value && !std::is_same<Ti, gml_list_t>::value, bool> = true>
  std::string to_string() const
  {
    std::stringstream ss;
    ss << _value;
    return ss.str();
  }

  virtual operator std::string() const override
  {
    return to_string();
  }

private:
  T _value;
};

struct gml_list_t
{
  template <typename Key, typename Value>
  void emplace(const Key& key, const Value& value)
  {
    _keys.emplace_back(key);
    _values.push_back(std::make_shared<gml_generic_value_t<Value>>(value));
  }
  friend std::ostream& operator<<(std::ostream& os, const gml_list_t& obj)
  {
    prx_assert(obj._keys.size() == obj._values.size(), "Mismatch in keys and values");

    for (int i = 0; i < obj._keys.size(); ++i)
    {
      os << obj._keys[i];
      os << " ";
      os << *obj._values[i];
      os << "\n";
    }
    return os;
  }

private:
  std::vector<gml_key_t> _keys;
  std::vector<std::shared_ptr<gml_value_t>> _values;
};

}  // namespace utilities
}  // namespace prx