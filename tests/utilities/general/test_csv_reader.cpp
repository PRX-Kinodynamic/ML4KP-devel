#define BOOST_AUTO_TEST_MAIN csv_reader_tests
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
  using Block = csv_reader_t::Block<std::string>;
  csv_reader_t reader(test_file, ' ');
  while (reader.has_next_line())
  {
    Block block{ reader.next_block() };
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
    auto line = reader.next_line<std::string>("block1", 0);
  }
  BOOST_CHECK(total_lines == 3);
}

BOOST_AUTO_TEST_CASE(csv_reader_reads_lines_with_custom_function)
{
  using Line = csv_reader_t::Line<std::string>;
  csv_reader_t reader(test_file, ' ');
  std::size_t total_lines{ 0 };
  while (reader.has_next_line())
  {
    Line line{ reader.next_line([](const Line& line) { return line.size() > 2 && line[2] == "4"; }) };
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
BOOST_AUTO_TEST_CASE(read_column_test)
{
  csv_reader_t reader(test_file, ' ');
  const std::size_t column_to_read{ 2 };
  const std::vector<int> expected_column = { 1, 4, 7, 1, 4, 7 };

  std::vector<int> column_read = reader.read_column<int>(column_to_read);

  BOOST_REQUIRE(column_read.size() == expected_column.size());
  for (int i = 0; i < expected_column.size(); ++i)
  {
    BOOST_CHECK_MESSAGE(expected_column[i] == column_read[i], EXPECTED_GOT(expected_column[i], column_read[i]));
  }
}

BOOST_AUTO_TEST_CASE(read_columns_test)
{
  using Columns = std::vector<std::vector<int>>;
  csv_reader_t reader(test_file, ' ');
  const std::vector<std::size_t> columns_to_read = { 2, 3, 4 };
  const Columns expected_columns = { { 1, 4, 7, 1, 4, 7 },  // no-lint
                                     { 2, 5, 8, 2, 5, 8 },  // no-lint
                                     { 3, 6, 9, 3, 6, 9 } };

  const Columns columns_read = reader.read_columns<int>(columns_to_read);

  BOOST_REQUIRE(columns_read.size() == expected_columns.size());
  for (int i = 0; i < expected_columns.size(); ++i)
  {
    BOOST_REQUIRE(columns_read[i].size() == expected_columns[i].size());
    for (int j = 0; j < expected_columns[i].size(); ++j)
    {
      BOOST_CHECK_MESSAGE(expected_columns[i][j] == columns_read[i][j],
                          EXPECTED_GOT(expected_columns[i][j], columns_read[i][j]));
    }
  }
}
