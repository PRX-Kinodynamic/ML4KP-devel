#define BOOST_AUTO_TEST_MAIN thread_pool_test
#include <string>
// TODO: Change to <boost/test/unit_test.hpp>
#include <boost/test/unit_test.hpp>

#include "prx/external/thread_pool/BS_thread_pool.hpp"

int the_answer()
{
  return 42;
}
BOOST_AUTO_TEST_CASE(test_thread_pool_lib)
{
  BS::thread_pool pool;
  std::future<int> my_future{ pool.submit_task(the_answer) };
  BOOST_CHECK(my_future.get() == 42);
}
