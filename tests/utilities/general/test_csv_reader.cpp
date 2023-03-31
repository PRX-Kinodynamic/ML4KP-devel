#define BOOST_AUTO_TEST_MAIN abstract_node
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/csv_reader.hpp"

using csv_reader_t = prx::utilities::csv_reader_t;
const std::string test_file{ prx::lib_path + "tests/utilities/general/test_csv_reader.txt" };
/** test_file:
block0 line0 1 2 3
block0 line1 4 5 6
block0 line2 7 8 9

block1 line0 1 2 3 more_data
block1 line1 4 5 6 more_data
block1 line2 7 8 9 more_data
**/

BOOST_AUTO_TEST_CASE(csv_reader_opens_correctly)
{
  csv_reader_t reader(test_file, ' ');
  BOOST_CHECK_MESSAGE(reader.is_open(), "File was not opened");
  BOOST_CHECK_MESSAGE(reader.has_next_line(), "File has no new line");
}

BOOST_AUTO_TEST_CASE(csv_reader_reads_all_lines)
{
  csv_reader_t reader(test_file, ' ');
  std::size_t total_lines{ 0 };
  while (reader.has_next_line())
  {
    total_lines++;
    auto line = reader.next_line();
  }
  BOOST_CHECK(total_lines == 7);
}

BOOST_AUTO_TEST_CASE(csv_reader_reads_all_blocks)
{
  csv_reader_t reader(test_file, ' ');
  std::size_t total_blocks{ 0 };
  while (reader.has_next_line())
  {
    total_blocks++;
    auto block = reader.next_block();
  }
  BOOST_CHECK(total_blocks == 2);
}

BOOST_AUTO_TEST_CASE(csv_reader_reads_blocks_without_empty_line)
{
  csv_reader_t reader(test_file, ' ');
  while (reader.has_next_line())
  {
    csv_reader_t::Block block = reader.next_block();
    for (auto line : block)
    {
      BOOST_CHECK(line.size() > 0);
    }
  }
}

BOOST_AUTO_TEST_CASE(csv_reader_reads_lines_with_value)
{
  csv_reader_t reader(test_file, ' ');
  std::size_t total_lines{ 0 };
  while (reader.has_next_line())
  {
    total_lines++;
    auto line = reader.next_line("block1", 0);
  }
  BOOST_CHECK(total_lines == 3);
}

BOOST_AUTO_TEST_CASE(csv_reader_reads_lines_with_custom_function)
{
  csv_reader_t reader(test_file, ' ');
  std::size_t total_lines{ 0 };
  while (reader.has_next_line())
  {
    auto line = reader.next_line([](const csv_reader_t::Line& line) { return line.size() > 2 && line[2] == "4"; });
    if (line.size() > 0)
      total_lines++;
  }
  BOOST_CHECK(total_lines == 2);
}

BOOST_AUTO_TEST_CASE(csv_reader_iterator_iterates)
{
  csv_reader_t reader(test_file, ' ');
  std::size_t total_lines{ 0 };
  const std::size_t expected_total_lines{ 7 };
  for (auto line : reader)
  {
    // Avoid infinite loop
    BOOST_CHECK(total_lines < expected_total_lines);
    total_lines++;
  }
  BOOST_CHECK(total_lines == expected_total_lines);
}