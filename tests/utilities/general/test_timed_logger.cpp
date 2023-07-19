#define BOOST_AUTO_TEST_MAIN timed_logger_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/timed_logger.hpp"

BOOST_AUTO_TEST_CASE(timed_logger_integer_test)
{
  using namespace std::literals::chrono_literals;
  // Build, add a variable and run
  std::stringstream ss;
  prx::utilities::timed_logger_t timed_logger(500ms, ss);
  int integer = 0;
  timed_logger.add("integer", &integer);
  timed_logger.run();

  std::this_thread::sleep_for(1000ms);
  integer = 5;
  std::this_thread::sleep_for(1000ms);

  timed_logger.stop();

  std::istringstream iss(ss.str());

  std::string first{ "" };
  std::string last{ "" };
  for (std::string line; std::getline(iss, line);)
  {
    if (first == "")
    {
      first = line;
    }
    last = line;
    // std::cout << "line:" << line;
  }
  BOOST_CHECK_MESSAGE(first == "integer 0", "Got: " << first);
  BOOST_CHECK_MESSAGE(last == "integer 5", "Got: " << last);
}

BOOST_AUTO_TEST_CASE(timed_logger_multiple_types_test)
{
  using namespace std::literals::chrono_literals;
  // Build, add a variable and run
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2);
  prx::utilities::timed_logger_t timed_logger(500ms, ss);
  int integer = 0;
  double double_num = 0;
  Eigen::Vector2d eigen_vector{ Eigen::Vector2d::Zero() };
  timed_logger.add("integer", &integer);
  timed_logger.add("double_num", &double_num);
  auto nice_printer = [&]() { return eigen_vector.transpose(); };
  timed_logger.add("eigen_vector", nice_printer);
  timed_logger.run();

  std::this_thread::sleep_for(1000ms);
  integer = 5;
  double_num = 3.14;
  eigen_vector[0] = 1;
  eigen_vector[1] = 2;
  std::this_thread::sleep_for(1000ms);

  timed_logger.stop();

  // std::cout << ss.str();
  std::istringstream iss(ss.str());

  std::vector<std::string> first;
  std::vector<std::string> last;
  for (std::string line; std::getline(iss, line);)
  {
    first.push_back(line);
    last.emplace(last.begin(), line);
  }
  BOOST_REQUIRE(first.size() >= 3);
  BOOST_REQUIRE(last.size() >= 3);

  BOOST_CHECK_MESSAGE(first[0] == "integer 0", "Got: " << first[0]);
  BOOST_CHECK_MESSAGE(first[1] == "double_num 0", "Got: " << first[1]);
  BOOST_CHECK_MESSAGE(first[2] == "eigen_vector 0 0", "Got: " << first[2]);

  BOOST_CHECK_MESSAGE(last[2] == "integer 5", "Got: " << last[2]);
  BOOST_CHECK_MESSAGE(last[1] == "double_num 3.14", "Got: " << last[1]);
  BOOST_CHECK_MESSAGE(last[0] == "eigen_vector 1 2", "Got: " << last[0]);
  // BOOST_CHECK_MESSAGE(first == "integer 0", "Got: " << first);
  // BOOST_CHECK_MESSAGE(last == "integer 5", "Got: " << last);
}