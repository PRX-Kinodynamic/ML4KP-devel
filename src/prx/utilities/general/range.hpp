#pragma once
#include <tuple>
#include <limits>
#include "prx/utilities/defs.hpp"

namespace prx
{
template <typename Type>
class range_t;

typedef range_t<std::size_t> index_range_t;

/**
 * @author Edgar Granados
 *
 * A C++ version of python's range()
 * This class uses iterators to go over a range of numbers. The range number types
 * can be specified via the Type template. The start, end and increment via different
 * constructors.
 **/
template <typename Type>
class range_t
{
public:
  range_t(const Type start, const Type end, const Type increment) : _current(start), _end(end), _increment(increment)
  {
    _last = new range_t(*this);
    _last->_current = end;
  }
  range_t() : range_t(0, std::numeric_limits<Type>::max(), 1)
  {
  }
  range_t(const Type start) : range_t(start, std::numeric_limits<Type>::max(), 1)
  {
  }
  range_t(const Type start, const Type end) : range_t(start, end, 1)
  {
  }
  range_t(const range_t& _other) = default;

  struct range_iterator_t
  {
    using iterator_category_t = std::forward_iterator_tag;
    using difference_type_t = std::ptrdiff_t;
    using value_type_t = Type;
    using pointer_t = value_type_t*;
    using reference_t = value_type_t&;

    range_iterator_t() = delete;

    range_iterator_t(range_t* t)
    {
      m_ptr = t;
    }

    reference_t operator*() const
    {
      return m_ptr->_current;
    }
    pointer_t operator->()
    {
      return &(m_ptr->_current);
    }

    // Prefix increment
    range_iterator_t operator+(Type b)
    {
      m_ptr->_current += b;
      return *this;
    }

    // Prefix increment
    range_iterator_t& operator++()
    {
      (*this) = (*this) + m_ptr->_increment;
      return *this;
    }

    // Postfix increment
    range_iterator_t operator++(int)
    {
      // range_iterator_t tmp = *this;
      // ++(*this);
      // return tmp;
      return *this;
    }

    bool operator==(const range_iterator_t& other)
    {
      return m_ptr->_current == other.m_ptr->_current &&      // no-lint
             m_ptr->_increment == other.m_ptr->_increment &&  // no-lint
             m_ptr->_last == other.m_ptr->_last;
    };
    bool operator!=(const range_iterator_t& other)
    {
      return !((*this) == other);
    };

  private:
    range_t* m_ptr;
  };

  range_iterator_t begin()
  {
    return range_iterator_t(this);
  }
  range_iterator_t end()
  {
    return range_iterator_t(_last);
  }

protected:
  Type _current;
  Type _end;
  Type _increment;

  range_t<Type>* _last;
  friend range_iterator_t;
};

}  // namespace prx