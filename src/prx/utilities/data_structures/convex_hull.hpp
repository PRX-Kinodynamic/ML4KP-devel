#pragma once

#include "prx/utilities/defs.hpp"

#include "prx/utilities/spaces/space.hpp"

using namespace prx;

class convex_hull_t
{
public:
    std::vector<space_point_t> hull;

    static bool compare(const space_point_t& a, const space_point_t& b)
    {
        return a -> at(0) < b -> at(0);
    }

    int test_side(const space_point_t& a, const space_point_t& b, const space_point_t& c);

    double distance_point_line(const space_point_t& a, const space_point_t& b, const space_point_t& c);

    void find_hull(const std::vector<space_point_t>& points, const space_point_t& a, const space_point_t& b);

    std::vector<space_point_t> compute_hull(const std::vector<space_point_t>& points);
};