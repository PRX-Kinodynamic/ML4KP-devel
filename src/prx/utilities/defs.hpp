#include "prx/utilities/general/prx_assert.hpp"
#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/random.hpp"
#include "prx/utilities/general/transforms.hpp"
#include "prx/utilities/general/string_manip.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/general/zipped_iter.hpp"
#include "prx/utilities/general/progress_bar.hpp"
#include "prx/utilities/general/template_utils.hpp"

#define PRX_DEBUG_PRINT std::cout << __PRETTY_FUNCTION__ << ": " << __LINE__ << std::endl;

#define PRX_NOT_IMPLEMENTED                                                                                            \
  std::cout << prx::constants::color::red << __PRETTY_FUNCTION__ << " NOT IMPLEMENTED. Is an override needed?"         \
            << prx::constants::color::normal << std::endl;

#define PRX_DEBUG_ITERABLE(msg, v)                                                                                     \
  std::cout << "[DBG " << msg << "] ";                                                                                 \
  for (auto e : v)                                                                                                     \
  {                                                                                                                    \
    std::cout << e << " ";                                                                                             \
  }                                                                                                                    \
  std::cout << std::endl;

#define PRX_DEBUG_VAR_1(VAR) std::cout << #VAR << ": " << VAR << std::endl;
#define PRX_DEBUG_VAR_2(VAR1, VAR2) std::cout << #VAR1 << ": " << VAR1 << "\t" << #VAR2 << ": " << VAR2 << std::endl;
#define PRX_DEBUG_VAR_3(VAR1, VAR2, VAR3)                                                                              \
  std::cout << #VAR1 << ": " << VAR1 << "\t" << #VAR2 << ": " << VAR2 << "\t" << #VAR3 << ": " << VAR3 << std::endl;

#define GET_MACRO(_1, _2, _3, NAME, ...) NAME
#define PRX_DEBUG_VARS(...) GET_MACRO(__VA_ARGS__, PRX_DEBUG_VAR_3, PRX_DEBUG_VAR_2, PRX_DEBUG_VAR_1)(__VA_ARGS__)

#define STR_TO_BOOL(VAL) (std::string(VAL) == "True" | std::string(VAL) == "true")
#define STR_TO_INT(VAR) (std::stoi(VAR))
#define STR_TO_DOUBLE(VAR) (std::stod(VAR))

/**
 * Convert VAR to int and test VAR against CHECK. Eg. STR_TO_INT_AND_CHECK(foo, >= 0) ==> if(foo >= 0): parse(foo) else:
 * THROW
 * @param  VAR      String to parse to int
 * @param  CHECK    Check to perform
 * @return          The parse value or throws error if check is not passed.
 */
#define STR_TO_INT_AND_CHECK(VAR, CHECK)                                                                               \
  STR_TO_INT(VAR)                                                                                                      \
  CHECK ?                                                                                                              \
      STR_TO_INT(VAR) :                                                                                                \
      throw prx_assert_t(#CHECK, __FILE__, __LINE__, (prx_assert_t::stream_t() << "STR TO INT - CHECK not passed!"))
#define STR_TO_DOUBLE_AND_CHECK(VAR, CHECK)                                                                            \
  STR_TO_DOUBLE(VAR)                                                                                                   \
  CHECK ? STR_TO_DOUBLE(VAR) :                                                                                         \
          throw prx_assert_t(#CHECK, __FILE__, __LINE__,                                                               \
                             (prx_assert_t::stream_t() << "STR TO DOUBLE - CHECK not passed!"))

// Useful for tests
#define EXPECTED_GOT(expected, got) "Expected: " << expected << ". Got: " << got

#define PRX_DEPRECATED(MSG)                                                                                            \
  static bool deprecated_print_once = []() {                                                                           \
    std::cout << prx::constants::color::yellow << "[DEPRECATED] \"" << __PRETTY_FUNCTION__ << "\":" << MSG             \
              << prx::constants::color::normal << "\n";                                                                \
    return true;                                                                                                       \
  }();
