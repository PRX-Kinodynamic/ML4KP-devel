#include "prx/utilities/general/param_loader.hpp"
#include <ostream>
#include "general/debug_utils.hpp"
#include "prx/utilities/general/string_manip.hpp"

namespace prx
{

param_loader::param_loader(const std::string filename, const std::string path)
{
  set_input_path(path);
  add_file(filename);
}

param_loader::param_loader()
{
  set_input_path(input_path);
}
param_loader::param_loader(iterator first, iterator last)
{
  while (first != last)
  {
    params.push_back(*first);
    first++;
  }
}
param_loader::param_loader(const_iterator first, const_iterator last)
{
  while (first != last)
  {
    params.push_back(*first);
    first++;
  }
}

param_loader::param_loader(const std::string filename) : param_loader(filename, "") {};

param_loader::param_loader(int argc, char* argv[]) : param_loader(std::vector<std::string>(argv, argv + argc)) {};

param_loader::param_loader(std::vector<std::string> argv) : param_loader()
{
  add_opts(argv);
}

param_loader::param_loader(std::string filename, int argc, char* argv[])
  : param_loader(filename, std::vector<std::string>(argv, argv + argc)) {};

param_loader::param_loader(std::string filename, std::vector<std::string> argv) : param_loader(filename)
{
  add_opts(argv);
}

param_loader::param_loader(const param_loader& pl)
{
  set_input_path(input_path);
  params = std::move(pl.params);
}

param_loader::param_loader(YAML::Node input_params, std::string _p_key)
{
  p_key = _p_key;
  params = std::move(input_params);
}

void param_loader::add(const param_loader& pl)
{
  params.push_back(std::move(pl.params));
}

void param_loader::from_string(const std::string str)
{
  YAML::Node nn;
  try
  {
    // PRX_DBG_VARS(str);
    std::istringstream istrstr(str);
    nn = YAML::Load(istrstr);
  }
  catch (...)
  {
    prx_throw("[param_loader::from_string] Couldn't load string: " << str);
  }
  params = std::move(expand_file(nn));
}

void param_loader::add_file(std::string file_name)
{
  std::string filename;
  YAML::Node nn;
  try
  {
    filename = prx::check_which_file_exists(file_name, pl_input_path + file_name);
    nn = YAML::LoadFile(filename);
  }
  catch (...)
  {
    if (params.IsNull())
      prx_throw("Bad filename to param_loader '" << file_name << "'");
  }
  // try
  // {
  params = std::move(expand_file(nn));
  // }
  // catch (...)
  // {
  //   prx_throw("[param_loader] expand_file failed");
  // }
}

YAML::Node param_loader::expand_file(YAML::Node& node)
{
  YAML::Node new_node;
  // Do nothing if node has nothing
  if (!node.IsDefined())
    return new_node;

  if (node.Tag() == "!file")
  {
    try
    {
      replace_env_var(node);
      new_node = param_loader(node.as<std::string>()).params;
    }
    catch (...)
    {
      const std::string yaml_string{ YAML::Dump(node) };
      prx_throw("[param_loader] Failed to load '!file' " << yaml_string);
    }
  }
  else
  {
    for (auto p : node)
    {
      auto expanded = expand_file(p.second);
      if (!expanded.IsNull())
      {
        replace_env_var(expanded);
        node[p.first.as<std::string>()] = std::move(expanded);
      }
    }
    new_node = std::move(node);
  }
  return new_node;
}

YAML::Node param_loader::find(const std::string& key, YAML::Node node)
{
  YAML::Node n;
  for (auto p : node)
  {
    if (p.first.as<std::string>() == key)
    {
      return std::move(node[p.first.as<std::string>()]);
    }
    if (p.second.IsMap())
    {
      n = std::move(find(key, p.second));
    }
  }
  // return a null node if nothing was found
  return std::move(n);
}

void param_loader::add_opts(int argc, char* argv[])
{
  add_opts(std::vector<std::string>(argv, argv + argc));
}

void param_loader::add_opts(std::vector<std::string> argv)
{
  // Regular case: "--/some/param/name=value"
  const std::regex opt_regex_mult("--((\\/)?\\w)+=(.)+");
  // Bool can be without value: "--/some/bool/param"
  const std::regex opt_regex_bool("--((\\/)?\\w)+=?");
  // Special case for the executable: "./executable_name"
  const std::regex opt_regex_exec("(.)+(\\/\\w+)+");
  const std::regex opt_regex_expy("(.)+\\.py");

  auto argc = argv.size();
  for (int i = 0; i < argc; ++i)
  {
    std::string opt = argv[i];

    if (std::regex_match(opt, opt_regex_exec))
    {
      (*this)["executable"] = opt.substr(2);
    }
    else if (std::regex_match(opt, opt_regex_expy))
    {
      // std::cout << "opt: " << opt << std::endl;
      (*this)["executable"] = opt;
    }
    else if (std::regex_match(opt, opt_regex_mult))
    {
      // std::cout << "multi opt: " << opt << std::endl;

      (*this)[opt.substr(2, opt.find("=") - 2)] = YAML::Load(opt.substr(opt.find("=") + 1));
    }
    else if (std::regex_match(opt, opt_regex_bool))
    {
      // std::cout << "bool opt: " << opt << std::endl;
      auto pos_eq = opt.find("=");
      if (pos_eq != std::string::npos)
      {
        // opt = opt.substr(2, pos_eq - 2);
        pos_eq = pos_eq - 2;
      }
      (*this)[opt.substr(2, pos_eq)] = YAML::Load("true");
    }
    else if (i == 0)
    {  // Skip the first()
    }
    else
    {
      prx_warn("param_loader - Error reading param: " << opt);
    }
  }
}

void param_loader::merge(const YAML::Node& other)
{
  for (auto p : other)
  {
    params[p.first.as<std::string>()] = std::move(p.second);
  }
}

const param_loader param_loader::operator[](const std::string& key) const
{
  std::string::size_type subkey_init = key[0] == '/' ? 1 : 0;
  std::string::size_type subkey_pos = 0;  // = key.find("/", subkey_init);

  subkey_pos = key.find("/", subkey_init);
  // if(!params[key.substr(subkey_init, subkey_pos - subkey_init)])
  // {
  // 	prx_throw("Tried to access element \""<<key<<"\" which isn't there.");
  // }
  //
  // YAML::Node new_node;// = params[key.substr(subkey_init, subkey_pos - subkey_init)];
  auto new_node = params[key.substr(subkey_init, subkey_pos - subkey_init)];
  // p_key = key.substr(subkey_init, subkey_pos - subkey_init);
  if (subkey_pos == std::string::npos)
  {
    // if(!new_node.IsDefined())
    // if(!params[key.substr(subkey_init, subkey_pos - subkey_init)])
    // {
    // 	prx_throw("Tried to access element \""<<key<<"\" which isn't there.");
    // }
    // new_node =
    return param_loader(new_node, key.substr(subkey_init, subkey_pos - subkey_init));
  }
  // Tail recursive!
  return param_loader(new_node, key.substr(subkey_init, subkey_pos - subkey_init))[key.substr(subkey_pos)];
}

param_loader param_loader::operator[](const std::string& key)
{
  std::string::size_type subkey_init = key[0] == '/' ? 1 : 0;
  std::string::size_type subkey_pos = 0;  // = key.find("/", subkey_init);
  // YAML::Node new_node = params;

  // if(!params[key.substr(subkey_init, subkey_pos - subkey_init)])
  // {
  // 	prx_throw("Tried to access element \""<<key<<"\" which isn't there.");
  // }
  // std::cout << "key: " << key << std::endl;
  subkey_pos = key.find("/", subkey_init);
  // std::cout << "\tsubkey: " << key.substr(subkey_init, subkey_pos - subkey_init) << " init: " << subkey_init << "
  // pos: " << subkey_pos << std::endl;
  auto new_node = params[key.substr(subkey_init, subkey_pos - subkey_init)];

  p_key = key.substr(subkey_init, subkey_pos - subkey_init);
  if (subkey_pos == std::string::npos)
  {
    return param_loader(new_node, key.substr(subkey_init, subkey_pos - subkey_init));
  }
  else
  {
    // Tail recursive!
    return param_loader(new_node, key.substr(subkey_init, subkey_pos - subkey_init))[key.substr(subkey_pos)];
  }
}

void param_loader::print() const
{
  print(params);
}

void param_loader::print(const YAML::Node& pl, std::string prepath) const
{
  bool nf = false;
  switch (pl.Type())
  {
    case YAML::NodeType::Undefined:
      printf(": Undefined\n");
      break;
    case YAML::NodeType::Null:
      printf("%s\n", prepath.c_str());
      break;
    case YAML::NodeType::Scalar:
      printf("%s: %s\n", prepath.c_str(), pl.as<std::string>().c_str());
      break;
    case YAML::NodeType::Sequence:
      printf("%s: [", prepath.c_str());
      for (auto p : pl)
      {
        if (nf)
          printf(",");
        printf(" %s", p.as<std::string>().c_str());
        nf = true;
      }
      printf(" ]\n");
      break;
    case YAML::NodeType::Map:
      for (auto p : pl)
      {
        // printf("/%s", p.first.as<std::string>().c_str());
        // printf("tag: %s\n", p.second.Tag().c_str());
        print(p.second, prepath + "/" + p.first.as<std::string>());
      }
      break;
    default:
      prx_throw("Problem printing param_loader! Is there a new type?\n");
  }
}

void param_loader::replace_env_var(YAML::Node& node)
{
  const std::regex env_var_regex("\\$\\{(.)+\\}");
  // std::cout << "Map:" << node.IsMap() << std::endl;
  // std::cout << "Sequence:" << node.IsSequence() << std::endl;
  if (!node.IsSequence() && !node.IsMap())
  {
    const std::string node_str{ node.as<std::string>() };

    std::smatch regex_match;
    if (std::regex_search(node_str, regex_match, env_var_regex))
    {
      for (std::size_t i = 0; i < regex_match.size() - 1; i++)
      {
        const std::string match{ regex_match[i] };
        const std::string env_var_name{ match.substr(2, match.size() - 3) };
        char* value = std::getenv(env_var_name.c_str());
        prx_assert(value != NULL, "Env variable " << env_var_name << " not found!");
        const std::string replaced = std::regex_replace(node_str, env_var_regex, std::string(value));
        node = replaced;
      }
    }
  }
  else if (node.IsSequence())
  {
    for (auto n : node)
    {
      replace_env_var(n);
    }
  }
  else if (node.IsMap())
  {
    for (auto n : node)
    {
      replace_env_var(n.second);
    }
  }
}

}  // namespace prx