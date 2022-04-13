#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>

#include "prx/utilities/defs.hpp"

namespace prx 
{

    class prx_symbol_t
    {
        protected:
        // char1, char2, state
        uint8_t c1_, c2_, state_idx_; 
        uint64_t t_;

        private:
         /**
          * Constructor.
          *
          * @param[in] s         1 or 2 characters to represent the variable type
          * @param[in] state_idx  index of the state
          * @param[in] t         time step
          */
            prx_symbol_t(const std::string& s, uint8_t state_idx, uint64_t t);

        public:
            /** Default constructor */
            prx_symbol_t();

            /** Copy constructor */
            prx_symbol_t(const prx_symbol_t& key);

            /**
             * Constructor for symbol related to both link and joint.
             *  See private constructor
             */
            static prx_symbol_t state_symbol(uint64_t t)
            {
                return prx_symbol_t("Xi", 0, t);
            }

            static prx_symbol_t control_symbol(uint64_t t)
            {
                return prx_symbol_t("Ui", 0, t);
            }

            static prx_symbol_t goal_state_symbol()
            {
                return prx_symbol_t("Xg", 0, 0);
            }

            static std::string state_string_symbol(std::string c, int i)
            {
                std::string str(c);
                str.push_back(static_cast<char>(i + 'a'));
                return str;
            }
            /**
             * Constructor for symbol related to neither joint or link (e.g. time).
             *
             * @param[in] s         1 or 2 characters to represent the variable type
             * @param[in] t         time step
             */
// prx_symbol_t prx_symbol_t::time_symbol(const std::string& s, uint64_t t) {
            static prx_symbol_t time_symbol(uint64_t t)
            {
                return prx_symbol_t("ti", 0, t);
            }

            /**
             * Constructor that decodes an integer gtsam::Key
             */
            prx_symbol_t(const gtsam::Key& key);

            /// Cast to a GTSAM Key.
            operator gtsam::Key() const;

            /// Return string label.
            std::string label() const;

            /// Return link id.
            inline uint8_t state_Idx() const { return state_idx_; }

            /// Retrieve key index.
            inline uint64_t time() const { return t_; }

            /// Print.
            void print(const std::string& s = "") const;

            /// Check equality.
            bool equals(const prx_symbol_t& expected, double tol = 0.0) const 
            {
              return (*this) == expected;
            }

            /// return the integer version
            gtsam::Key key() const { return (gtsam::Key) * this; }

            /// Create a string from the key
            operator std::string() const;

        private:

  /**
   * \defgroup Bitfield bit field constants
   * @{
   */
  static constexpr size_t kMax_uchar_ =
      std::numeric_limits<uint8_t>::max();
    static constexpr size_t kMax_state_ = 
      std::numeric_limits<uint8_t>::max();
  // bit counts
  static constexpr size_t key_bits = sizeof(gtsam::Key) * 8;
  static constexpr size_t ch1_bits = sizeof(uint8_t) * 8;
  static constexpr size_t ch2_bits = sizeof(uint8_t) * 8;
  static constexpr size_t state_bits = sizeof(uint8_t) * 8;
  static constexpr size_t time_bits = key_bits - ch1_bits - ch2_bits - state_bits;
  // masks
  static constexpr gtsam::Key ch1_mask = gtsam::Key(kMax_uchar_)
                                         << (key_bits - ch1_bits);
  static constexpr gtsam::Key ch2_mask = gtsam::Key(kMax_uchar_)
                                         << (key_bits - ch1_bits - ch2_bits);
  static constexpr gtsam::Key state_mask = gtsam::Key(kMax_state_ )
                                          << (key_bits - ch1_bits - ch2_bits - state_bits);
  static constexpr gtsam::Key time_mask =
      ~(ch1_mask | ch2_mask | state_mask);
  /**@}*/
};

// /// key formatter function
    std::string key_formatter(gtsam::Key key);

// static const gtsam::KeyFormatter GTDKeyFormatter = &_GTDKeyFormatter;

}  // namespace gtdynamics
