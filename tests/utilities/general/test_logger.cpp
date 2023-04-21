#define BOOST_TEST_MODULE logger_tests
#include <boost/test/included/unit_test.hpp>
#include <string>
#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/logger.hpp"
#include "prx/utilities/general/type_convertions.hpp"

using prx::logger_t;

BOOST_AUTO_TEST_CASE(logger_creates_file_correctly)
{
  const std::string test_file{ "/tmp/test.log" };
  logger_t log(test_file, ' ');

  const std::filesystem::path log_path{ test_file };
  BOOST_CHECK_MESSAGE(log.get_filename() == test_file, "Different name found");
  BOOST_CHECK_MESSAGE(std::filesystem::exists(log_path), "File not created");
  std::remove(test_file.c_str());
}

BOOST_AUTO_TEST_CASE(logger_logs_primitive_types_correctly)
{
  prx::separating_value = ' ';
  const std::string test_file{ "/tmp/test.log" };
  logger_t log(test_file, ' ');

  const std::string first{ "first" };  // string can be here but is not granted primitive-ness
  const int second{ -2 };
  const double third{ 3.1415 };
  const std::size_t fourth{ 4 };
  const bool fifth{ true };

  log(first, second, third, fourth, fifth);
  log.close();

  std::ifstream file(test_file.c_str());
  std::string line_str;
  std::getline(file, line_str);
  std::vector<std::string> line = prx::split<std::string>(line_str);

  BOOST_CHECK_MESSAGE(line[0] == first, EXPECTED_GOT(first, line[0]));
  BOOST_CHECK_MESSAGE(prx::utilities::convert_to<int>(line[1]) == second, EXPECTED_GOT(second, line[1]));
  BOOST_CHECK_MESSAGE(prx::utilities::convert_to<double>(line[2]) == third, EXPECTED_GOT(third, line[2]));
  BOOST_CHECK_MESSAGE(prx::utilities::convert_to<std::size_t>(line[3]) == fourth, EXPECTED_GOT(fourth, line[3]));
  BOOST_CHECK_MESSAGE(prx::utilities::convert_to<bool>(line[4]) == fifth, EXPECTED_GOT(fifth, line[4]));

  std::remove(test_file.c_str());
}

BOOST_AUTO_TEST_CASE(logger_logs_container_types_correctly)
{
  prx::separating_value = ' ';
  const std::string test_file{ "/tmp/test.log" };
  logger_t log(test_file, ' ');

  const std::vector<std::string> str_vec = { "a", "b" };
  const std::vector<int> int_vec = { 1, 2 };

  log(str_vec);
  log(int_vec);
  log.close();

  std::ifstream file(test_file.c_str());
  std::string line_str;
  std::getline(file, line_str);
  std::vector<std::string> res_str = prx::split<std::string>(line_str);
  std::getline(file, line_str);
  std::vector<int> res_int = prx::split<int>(line_str);

  BOOST_CHECK_MESSAGE(res_str[0] == str_vec[0], EXPECTED_GOT(str_vec[0], res_str[0]));
  BOOST_CHECK_MESSAGE(res_str[1] == str_vec[1], EXPECTED_GOT(str_vec[1], res_str[1]));
  BOOST_CHECK_MESSAGE(res_int[0] == int_vec[0], EXPECTED_GOT(int_vec[0], res_int[0]));
  BOOST_CHECK_MESSAGE(res_int[1] == int_vec[1], EXPECTED_GOT(int_vec[1], res_int[1]));

  std::remove(test_file.c_str());
}