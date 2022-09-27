#pragma once

// #include "prx/planning/planners/rrt.hpp"
#include "prx/utilities/defs.hpp"
#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>

namespace prx {
/**
 * A structure to store tree node pointers on a priority queue.
 * Providing an erase method which the std implementation do not.
 *
 * @Author: Edgar Granados
 */
typedef std::function<bool(const std::shared_ptr<rrt_node_t> &,
                           const std::shared_ptr<rrt_node_t> &)>
    compare_function_t;
typedef std::function<bool(const std::shared_ptr<rrt_node_t> &,
                           const std::shared_ptr<rrt_node_t> &)>
    equivalent_function_t;

template <class T = std::shared_ptr<rrt_node_t>, class Container = std::list<T>,
          class Compare = compare_function_t,
          class Equiv = equivalent_function_t>
class priority_queue_t {

public:
  priority_queue_t() {
    container.clear();
    cmp_function = [](const std::shared_ptr<rrt_node_t> &n1,
                      const std::shared_ptr<rrt_node_t> &n2) {
      return n1->cost_to_come < n2->cost_to_come;
    };
    eq_function = [](const std::shared_ptr<rrt_node_t> &n1,
                     const std::shared_ptr<rrt_node_t> &n2) {
      // for (int i = 0; i < n1 -> point -> get_dim(); i++)
      // {
      // if (n1 -> point -> at(i) != n2 -> point -> at(i)) return false;
      // }
      // return true;
      // printf("%s:%d\n", __PRETTY_FUNCTION__, __LINE__ );
      // printf("n1 null? %s\n", n1 == nullptr?"true":"false" );
      // printf("n1 null? %s\n", n2 == nullptr?"true":"false" );
      return (n1 != nullptr) && (n2 != nullptr) &&
             n1->get_index() == n2->get_index();
    };
  }

  priority_queue_t(Compare cmp, Equiv eq) {
    container.clear();
    eq_function = eq;
  }

  priority_queue_t(std::vector<T> v, Compare cmp, Equiv eq) {
    container = v;
    cmp_function = cmp;
    cmp_function = eq;
  }
  priority_queue_t(std::vector<T> v) {
    container = v;
    cmp_function = [](const std::shared_ptr<rrt_node_t> &n1,
                      const std::shared_ptr<rrt_node_t> &n2) {
      return n1->cost_to_come < n2->cost_to_come;
    };
    eq_function = [](const std::shared_ptr<rrt_node_t> &n1,
                     const std::shared_ptr<rrt_node_t> &n2) {
      // for (int i = 0; i < n1 -> point -> get_dim(); i++)
      // {
      //     if (n1 -> point -> at(i) != n2 -> point -> at(i)) return false;
      // }
      return n1->get_index() == n2->get_index();
    };
  }

  ~priority_queue_t() {
    // clear();
  }

  void push(T element) {
    auto it = std::lower_bound(container.begin(), container.end(), element,
                               cmp_function);
    container.insert(it, element);
  }

  T top() { return container.front(); }

  void pop() {
    container.pop_front();
    // container.erase(container.begin());
  }

  void erase(T element) {
    auto it_lower = std::lower_bound(container.begin(), container.end(),
                                     element, cmp_function);
    auto it_upper = std::upper_bound(container.begin(), container.end(),
                                     element, cmp_function);

    container.erase(it_lower, it_upper);
  }

  bool contains(T element) {
    auto it_lower = std::lower_bound(container.begin(), container.end(),
                                     element, cmp_function);
    auto it_upper = std::upper_bound(container.begin(), container.end(),
                                     element, cmp_function);

    for (auto i = it_lower; i != it_upper; ++i) {
      printf("Element %lu is equal to %lu? %s\n", element->get_index(),
             (*i)->get_index(), eq_function(element, *i) ? "true" : "false");
      if (eq_function(element, *i))
        return true;
    }
    return false;
    // return std::binary_search(container.begin(), container.end(), element,
    // cmp_function);
  }

  long unsigned size() const { return container.size(); }

  bool empty() const { return container.empty(); }

  void clear() { container.clear(); }

  void print_queue() {
    printf("Priority Queue: \n");
    bool first = true;
    for (auto e : container) {
      if (first)
        printf("%lu", e->get_index());
      else
        printf(", %lu", e->get_index());
      first = false;
    }
    printf("\n");
  }

private:
  Container container;
  Compare cmp_function;
  Equiv eq_function;
};
} // namespace prx
