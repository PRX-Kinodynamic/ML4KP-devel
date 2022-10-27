#include "prx/utilities/data_structures/convex_hull.hpp"

int convex_hull_t::test_side(const space_point_t& a, const space_point_t& b, const space_point_t& c)
{
    double val = (b -> at(1) - a -> at(1)) * (c -> at(0) - b -> at(0)) - (b -> at(0) - a -> at(0)) * (c -> at(1) - b -> at(1));
    if (val > 0)
        return 1;
    if (val < 0)
        return -1;
    return 0;
}

double convex_hull_t::distance_point_line(const space_point_t& a, const space_point_t& b, const space_point_t& c)
{
    return std::abs((b -> at(1) - a -> at(1)) * c -> at(0) - (b -> at(0) - a -> at(0)) * c -> at(1) + b -> at(0) * a -> at(1) - b -> at(1) * a -> at(0)) / std::sqrt(std::pow(b -> at(1) - a -> at(1), 2) + std::pow(b -> at(0) - a -> at(0), 2));
}

void convex_hull_t::find_hull(const std::vector<space_point_t>& points, const space_point_t& a, const space_point_t& b)
{
    if (points.empty()) return;

    int max_index = 0;
    double max_distance = 0;
    for (int i = 0; i < points.size(); i++)
    {
        double distance = distance_point_line(a, b, points[i]);
        if (distance > max_distance)
        {
            max_distance = distance;
            max_index = i;
        }
    }


    std::vector<space_point_t> left_points;
    std::vector<space_point_t> right_points;

    for (int i = 0; i < points.size(); i++)
    {
        if (test_side(a, points[max_index], points[i]) == 1)
            left_points.push_back(points[i]);
        else if (test_side(points[max_index], b, points[i]) == 1)
            right_points.push_back(points[i]);
    }

    find_hull(left_points, a, points[max_index]);
    hull.push_back(points[max_index]);
    find_hull(right_points, points[max_index], b);

    return;
}

std::vector<space_point_t> convex_hull_t::compute_hull(const std::vector<space_point_t>& points)
{
    hull.clear();

    if (points.size() <= 1) return points;

    // Sort points by x coordinate
    std::vector<space_point_t> sorted_points = points;
    std::sort(sorted_points.begin(), sorted_points.end(), compare);

    hull.push_back(sorted_points[0]);

    std::vector<space_point_t> left_points;
    std::vector<space_point_t> right_points;
    std::vector<space_point_t> on_line;

    for (int i = 1; i < sorted_points.size() - 1; i++)
    {
        if (test_side(sorted_points[0], sorted_points[sorted_points.size() - 1], sorted_points[i]) == 1)
            left_points.push_back(sorted_points[i]);
        else if (test_side(sorted_points[0], sorted_points[sorted_points.size() - 1], sorted_points[i]) == -1)
            right_points.push_back(sorted_points[i]);
        else
            on_line.push_back(sorted_points[i]);
    }

    if (left_points.size() == 0 || right_points.size() == 0)
    {
        for (int i = 0; i < on_line.size(); i++)
        {
            hull.push_back(on_line[i]);
        }
    }

    find_hull(left_points, sorted_points[0], sorted_points[sorted_points.size() - 1]);
    hull.push_back(sorted_points[sorted_points.size() - 1]);
    find_hull(right_points, sorted_points[sorted_points.size() - 1], sorted_points[0]);

    return hull;
}