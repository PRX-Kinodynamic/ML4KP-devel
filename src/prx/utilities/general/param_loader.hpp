#pragma once

#include <fstream>
#include <regex>
#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/prx_assert.hpp"
#include "prx/utilities/general/constants.hpp"

namespace prx
{
class param_loader
{
public:
  // typedef std::unordered_map<std::string, std::string>::iterator iterator;
  // typedef std::unordered_map<std::string, std::string>::const_iterator const_iterator;
  typedef YAML::Node::iterator iterator;
  typedef YAML::Node::const_iterator const_iterator;

  param_loader();
  param_loader(int argc, char* argv[]);
  param_loader(const std::string file_name);
  param_loader(const std::string file_name, const std::string path);
  param_loader(std::vector<std::string> argv);
  param_loader(std::string file_name, int argc, char* argv[]);
  param_loader(std::string file_name, std::vector<std::string> argv);
  param_loader(const param_loader& other);

  void add_file(std::string file_name);

  void add_opts(int argc, char* argv[]);
  void add_opts(std::vector<std::string> argv);

  // Load parameters from a string (NOT from a file)
  // As in str <- content of a file
  void from_string(const std::string str);

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

  const param_loader operator[](const std::string& key) const;

  param_loader operator[](const std::string& key);

  void add(const param_loader& pl);

  template <typename T>
  void set(T val)
  {
    params = val;
  }

  void print() const;

  inline bool exists(const std::string& key) const
  {
    std::string::size_type subkey_pos{ key.find("/", 0) };
    if (subkey_pos == std::string::npos and params[key])
      return true;
    if (params[key.substr(0, subkey_pos)])
    {
      return exists(key.substr(0, subkey_pos));
    }

    return false;
  }

  void replace_env_var(YAML::Node& node);

  void replace_environment_variables()
  {
    // for (auto p : params)
    // {
    replace_env_var(params);
    // }
  }

  template <typename T = std::string>
  T as() const
  {
    T val;
    try
    {
      val = params.as<T>();
    }
    catch (...)
    {
      if (!params.IsDefined())
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
    this->params = rhs;
    return *this;
  }

  inline iterator begin()
  {
    return params.begin();
  }

  inline iterator end()
  {
    return params.end();
  }

  inline const_iterator begin() const
  {
    return params.begin();
  }

  inline const_iterator end() const
  {
    return params.end();
  }

  friend std::ostream& operator<<(std::ostream& os, const param_loader& obj)
  {
    os << obj.params;
    return os;
  }

  void save(const std::string filename) const
  {
    std::ofstream ofs(filename.c_str());
    ofs << params;
  }

  void merge(const param_loader& other)
  {
    merge(other.params);
    replace_environment_variables();
  }

protected:
  param_loader(YAML::Node input_params, std::string _p_key = "INVALID_KEY");

  YAML::Node expand_file(YAML::Node& node);

  YAML::Node find(const std::string& key, YAML::Node node);

  void print(const YAML::Node& pl, std::string prepath = "") const;

  YAML::Node params;

  // Needed to check if the key has been defined. YAML implementation
  // assumes that you check before calling as<>()...
  // Which produces verbose code and is not really intuitive.
  std::string p_key;

  void merge(const YAML::Node& other);

  // std::unordered_map<std::string, param_loader> params;
  std::string pl_input_path;
};
}  // namespace prx