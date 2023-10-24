#include "prx/utilities/general/file_utils.hpp"

namespace prx
{
namespace utilities
{
std::vector<std::vector<double>> read_vectors_from_file(const std::string path, const std::string delimiter)
{
  std::ifstream file(path);
  std::vector<std::vector<double>> dataset;
  std::string line = "";
  while (std::getline(file, line))
  {
    std::vector<double> row;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, delimiter[0]))
    {
      row.push_back(std::stod(cell));
    }
    dataset.push_back(row);
  }
  return dataset;
}
}  // namespace utilities
}  // namespace prx
