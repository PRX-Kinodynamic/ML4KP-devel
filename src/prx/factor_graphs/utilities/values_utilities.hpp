#pragma once
#include <fstream>
#include <gtsam/nonlinear/Values.h>
#include "prx/factor_graphs/utilities/symbols_factory.hpp"

namespace prx
{
namespace fg
{
using SF = prx::fg::symbol_factory_t;

template <typename Estimate>
void get_value_safe(gtsam::Values& values, Estimate& variable, const gtsam::Key& key)
{
  try
  {
    variable = values.at<Estimate>(key);
  }
  catch (gtsam::ValuesKeyDoesNotExist e)
  {
    const std::string msg{ "[EXCEPTION] Var:" + SF::formatter(e.key()) + "\n" };
    std::cout << msg << std::string(e.what()) << std::endl;
  }
}

template <typename Type>
void values_by_type_to_file(const gtsam::Values& values, const std::string filename,
                            const std::ios_base::openmode _mode = std::ofstream::trunc)
{
  std::ofstream ofs(filename.c_str(), _mode);
  for (auto pair : values.keys())
  {
    // auto val_ptr = pair->second;
    // auto ptr = dynamic_cast<const gtsam::GenericValue<Type>*>(&pair.value);
    // gtsam::Value value{ values.find(pair)->value };
    // auto ptr = dynamic_cast<const gtsam::GenericValue<Type>>(&value);

    try
    {
      const Type value{ values.at<Type>(pair) };
      ofs << SF::formatter(pair) << " ";
      ofs << value << "\n";
      // PRX_DBG_VARS(value);
      // ofs << values.at<Type>(pair.key) << "\n";
    }
    catch (...)
    {
    }
  }
  ofs.close();
}

}  // namespace fg
}  // namespace prx