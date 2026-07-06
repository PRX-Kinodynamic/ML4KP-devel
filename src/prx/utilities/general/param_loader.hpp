#pragma once

#include <fstream>
#include <memory>
#include <regex>
#include <string>
#include <unordered_map>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/yaml.h>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/prx_assert.hpp"
#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/gtsam_bridge.hpp"

namespace prx
{

// TODO: treat this class as a wrapper for the internal YAML::NODE
// aka: when a param_loader is returned (param[key]), it encapsulates the node so a change to the new
// param_loader (which is really a change to the node), changes the original param_loader.
class param_loader
{
public:
  // typedef std::unordered_map<std::string, std::string>::iterator iterator;
  // typedef std::unordered_map<std::string, std::string>::const_iterator const_iterator;
  using iterator = YAML::Node::iterator;
  using const_iterator = YAML::Node::const_iterator;

  param_loader();
  param_loader(int argc, char* argv[]);
  param_loader(const std::string file_name);
  param_loader(const std::string file_name, const std::string path);
  param_loader(std::vector<std::string> argv);
  param_loader(std::string file_name, int argc, char* argv[]);
  param_loader(std::string file_name, std::vector<std::string> argv);
  param_loader(const param_loader& other);
  param_loader(YAML::Node params, std::string _p_key = "");
  param_loader(std::shared_ptr<YAML::Node> params, std::string _p_key = "");
  param_loader(iterator first, iterator last);
  param_loader(const_iterator first, const_iterator last);

  void add_file(std::string file_name);

  void add_opts(int argc, char* argv[]);
  void add_opts(std::vector<std::string> argv);

  // Load parameters from a string (NOT from a file)
  // As in str <- read(file) -- The str contains the content of a file, NOT the name of the file
  void from_string(const std::string str);

  static param_loader create(const std::string str)
  {
    param_loader params;
    params.from_string(str);
    return std::move(params);
  }

  operator std::string() const
  {
    std::stringstream strstr;
    strstr << *this;
    return strstr.str();
  }

  inline const std::string get_input_path() const
  {
    return pl_input_path;
  }

  inline void set_input_path(const std::string& new_path)
  {
    pl_input_path = new_path;
    if (pl_input_path.back() != '/')
    {
      pl_input_path += "/";
    }
  }

  param_loader operator[](const std::string& key) const;

  param_loader operator[](const std::string& key);

  void add(const param_loader& pl);

  template <typename T>
  void set(T val)
  {
    _params = val;
  }

  void print() const;

  inline bool exists(const std::string& key) const
  {
    std::string::size_type subkey_pos{ key.find("/", 0) };
    if (subkey_pos == std::string::npos and _params[key])
      return true;
    if (_params[key.substr(0, subkey_pos)])
    {
      return exists(key.substr(0, subkey_pos));
    }

    return false;
  }

  void replace_env_var(YAML::Node& node);

  void replace_env_var(std::shared_ptr<YAML::Node> node)
  {
    replace_env_var(*node);
  }

  void replace_environment_variables()
  {
    // for (auto p : params)
    // {
    replace_env_var(_params);
    // }
  }

  template <typename Type>
  Type get_or_default(const std::string key, Type default_value)
  {
    if (exists(key))
    {
      return this->operator[](key).as<Type>();
    }
    this->operator[](key).set(default_value);

    return default_value;
  }

  template <typename T = std::string>
  T as() const
  {
    T val;
    try
    {
      val = _params.as<T>();
    }
    catch (...)
    {
      if (!_params.IsDefined())
      {
        // params.EnsureNodeExists();
        prx_throw("Param loader - problem using " << p_key);
      }
      // std::cout << (params.IsDefined()?"true":"false") << std::endl;
      // prx_throw_backtrace("Tried to convert to an incorrect type for: ");
    }
    return val;
  }

  template <typename T>
  param_loader& operator=(const T& rhs)
  {
    _params = rhs;
    return *this;
  }

  inline iterator begin()
  {
    return _params.begin();
  }

  inline iterator end()
  {
    return _params.end();
  }

  inline const_iterator begin() const
  {
    return _params.begin();
  }

  inline const_iterator end() const
  {
    return _params.end();
  }

  friend std::ostream& operator<<(std::ostream& os, const param_loader& obj)
  {
    os << obj._params;
    return os;
  }

  void save(const std::string filename) const
  {
    std::ofstream ofs(filename.c_str());
    ofs << _params;
    ofs.close();
  }

  void merge(const param_loader& other)
  {
    merge(other._params);
    replace_environment_variables();
  }

  template <typename Type>
  static void copy(Type& variable, const param_loader& params, const std::string& variable_name)
  {
    variable = params.exists(variable_name) ? params[variable_name].as<Type>() : variable;
  }

  template <typename T, std::enable_if_t<not prx::utilities::is_any_ptr<T>::value, bool> = true>
  static void create_file(const std::string filename, const T& type)
  {
    type.initialization_parameters().save(filename);
  }
  template <typename T, typename... Ts, std::enable_if_t<prx::utilities::is_any_ptr<T>::value, bool> = true>
  static void create_file(const std::string filename, const T type)
  {
    create_file(filename, *type);
  }

  template <typename Type>
  static void create_file(const std::string filename)
  {
    Type::init().save(filename);
  }

  std::vector<std::string> keys() const;

protected:
  YAML::Node expand_file(YAML::Node& node);

  YAML::Node find(const std::string& key, YAML::Node node);

  void print(const YAML::Node& pl, std::string prepath = "") const;

  YAML::Node _params;

  // Needed to check if the key has been defined. YAML implementation
  // assumes that you check before calling as<>()...
  // Which produces verbose code and is not really intuitive.
  std::string p_key;

  void merge(const YAML::Node& other);

  // std::unordered_map<std::string, param_loader> _*params;
  std::string pl_input_path;
};

}  // namespace prx
#define SET_VARIABLE(PARAM_LOADER, VARIABLE) prx::param_loader::copy(VARIABLE, PARAM_LOADER, #VARIABLE);
