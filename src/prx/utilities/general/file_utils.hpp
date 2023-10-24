#pragma once
#include <vector>
#include <fstream>
#include <string>
#include <sstream>

namespace prx
{
namespace utilities
{
std::vector<std::vector<double>> read_vectors_from_file(const std::string path, const std::string delimiter = ",");
}  // namespace utilities
}  // namespace prx