#pragma once
namespace prx {

/**
 * @brief Convert quaternion to yaw angle (rotation around Z axis)
 * 
 * @param quaternion Array of 4 doubles representing quaternion
 * @param scalar_first If true, quaternion is [w,x,y,z], if false it's [x,y,z,w]
 * @return double Yaw angle in radians
 */
double quaternion_to_yaw(const std::array<double, 4>& quaternion, bool scalar_first = false){
     // Extract w,x,y,z based on format
    double w = scalar_first ? quaternion[0] : quaternion[3];
    double x = scalar_first ? quaternion[1] : quaternion[0];
    double y = scalar_first ? quaternion[2] : quaternion[1];
    double z = scalar_first ? quaternion[3] : quaternion[2];

    // Extract yaw from quaternion using atan2
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
};

std::array<double, 4> yaw_to_quaternion(double yaw, bool scalar_first=false){
    if (scalar_first){
        return {std::cos(yaw/2.0), 0.0, 0.0, std::sin(yaw/2.0)};
    }
    else{
        return {0.0, 0.0, std::sin(yaw/2.0), std::cos(yaw/2.0)};
    }
}

/**
 * @brief Rotate a 2D point around a center point
 * 
 * @param px Point x coordinate
 * @param py Point y coordinate
 * @param center_x Center x coordinate
 * @param center_y Center y coordinate
 * @param angle Rotation angle in radians
 * @return std::array<double, 2> Rotated point coordinates
 */
std::array<double, 2> rotate_point(
    double px, double py, 
    double center_x, double center_y, 
    double angle){
        double cos_a = std::cos(angle);
        double sin_a = std::sin(angle);
        double dx = px - center_x;
        double dy = py - center_y;
        double rx = center_x + dx * cos_a - dy * sin_a;
        double ry = center_y + dx * sin_a + dy * cos_a;
        return {rx, ry};

    };

/**
 * @brief Get midpoint between two 2D points
 * 
 * @param point_1 First point
 * @param point_2 Second point
 * @return std::array<double, 2> Midpoint coordinates
 */
std::array<double, 2> get_mid_point(
    const std::array<double, 2>& point_1,
    const std::array<double, 2>& point_2){
        return {
        (point_1[0] + point_2[0]) / 2.0,
        (point_1[1] + point_2[1]) / 2.0
    };
};

} // namespace prx 