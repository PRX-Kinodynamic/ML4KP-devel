#pragma once

#include <deque>
#include <functional>
#include <memory>

namespace prx
{

/**
 * An object that preallocates std::shared_ptrs<Type> and when called returns one
 * avoiding paying for allocation every time a Type needs to be created.
 * The objective is to avoid paying for allocations which can be expensive by having
 * a buffer of preallocated objects.
 * @brief <b> A factory of preallocated std::shared_ptrs. </b>
 * @author Edgar Granados
 */

template <typename Type>
class pointer_factory_t
{
public:
  using TypePtr = std::shared_ptr<Type>;

  /**
   * @brief Constructor
   * @param state The first node to add to the structure.
   */
  pointer_factory_t(const std::size_t buffer_size) : _max_buffer_size(buffer_size)
  {
    allocate();
  };

  ~pointer_factory_t() {};

  void allocate()
  {
    // TODO: Use custom allocator for the pointers
    for (int i = _buffer.size(); i < _max_buffer_size; ++i)
    {
      _buffer.push_back(std::make_shared<Type>());
    }
  }

  TypePtr next()
  {
    if (_buffer.size() == 0)
    {
      allocate();
    }
    TypePtr ptr{ _buffer.front() };
    _buffer.pop_front();
    return ptr;
  }

private:
  std::deque<TypePtr> _buffer;
  const std::size_t _max_buffer_size;
};
}  // namespace prx