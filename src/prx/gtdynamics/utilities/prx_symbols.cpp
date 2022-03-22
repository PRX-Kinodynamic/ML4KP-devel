#include "prx/gtdynamics/utilities/prx_symbols.hpp"

#include <gtdynamics/utils/DynamicsSymbol.h>

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>

// using gtsam::Key;
namespace prx
{

/* ************************************************************************* */
// prx_symbol_t::prx_symbol_t()
    // : c1_(0), c2_(0), state_idx_(0), t_(0) {}

/* ************************************************************************* */
// prx_symbol_t::prx_symbol_t(const DynamicsSymbol& key)
//     : c1_(key.c1_),
//       c2_(key.c2_),
//       state_idx_(key.state_idx_),
//       t_(key.t_) {}

/* ************************************************************************* */
    prx_symbol_t::prx_symbol_t(const std::string& s, uint16_t state_idx, uint64_t t)
      : state_idx_(state_idx), t_(t) 
    {
        if (s.length() > 2) {
          throw std::runtime_error(
              "cannot use more than 2 characters in dynamics symbol");
        }
        if (s.length() > 1) 
        {
            c1_ = s[0];
            c2_ = s[1];
        } 
        else if (s.length() == 1) 
        {
            c1_ = 0;
            c2_ = s[0];
        } 
        else
        {
            c1_ = 0;
            c2_ = 0;
        }
    }


prx_symbol_t prx_symbol_t::time_symbol(const std::string& s, uint64_t t) {
  return prx_symbol_t(s, kMax_state_, t);
}

/* ************************************************************************* */
prx_symbol_t::prx_symbol_t(const gtsam::Key& key) {
  c1_ = (uint8_t)((key & ch1_mask) >> (key_bits - ch1_bits));
  c2_ = (uint8_t)((key & ch2_mask) >> (key_bits - ch1_bits - ch2_bits));
  state_idx_ = (uint16_t)((key & state_mask) >> (time_bits));
  t_ = key & time_mask;
}

/* ************************************************************************* */
    prx_symbol_t::operator gtsam::Key() const {
        gtsam::Key ch1_comp = gtsam::Key(c1_) << (key_bits - ch1_bits);
        gtsam::Key ch2_comp = gtsam::Key(c2_) << (key_bits - ch1_bits - ch2_bits);
        gtsam::Key state_comp = gtsam::Key(state_idx_) << (time_bits);
        gtsam::Key key = ch1_comp | ch2_comp | state_comp | t_;
        return key;
    }

/* ************************************************************************* */
std::string prx_symbol_t::label() const {
  std::string s = "";
  if (c1_ != 0) 
  {
    s += c1_;
  }
  if (c2_ != 0) 
  {
    s += c2_;
  }
  return s;
}

/* ************************************************************************* */
void prx_symbol_t::print(const std::string& s) const 
{
  if (s != "") {
    std::cout << s << ": ";
  }
  std::cout << std::string(*this) << std::endl;
}

/* ************************************************************************* */
prx_symbol_t::operator std::string() const 
{
  std::string s = label();
  if (state_idx_ != kMax_uchar_) {
    s += "[" + std::to_string((int)(state_idx_)) + "]";
  }
  s += std::to_string(t_);
  return s;
}

std::string _GTDKeyFormatter(gtsam::Key key) {
  return std::string(prx_symbol_t(key));
}

/* ************************************************************************* */

}  // namespace gtdynamics
