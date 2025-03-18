#pragma once
namespace prx {

// void wait_for_user_input(const std::string& message = "Press Enter to continue...") {
//     std::cout << message << std::flush;
//     std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
// }

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
    double siny_cosp = 2.0 * (w * z + x * 
    y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
};

/**
 * @brief Convert yaw angle to quaternion
 * 
 * @param yaw Yaw angle in radians
 * @param scalar_first If true, return [w,x,y,z], if false return [x,y,z,w]
 * @return std::array<double, 4> Quaternion
 */
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

/**
 * @brief Convert quaternion to rotation matrix
 * 
 * @param quaternion Quaternion [x,y,z,w] or [w,x,y,z]
 * @param scalar_first If true, quaternion is [w,x,y,z], if false it's [x,y,z,w]
 * @return std::array<std::array<double, 3>, 3> 3x3 rotation matrix
 */
std::array<std::array<double, 3>, 3> quaternion_to_rotation_matrix(const std::array<double, 4>& quaternion, bool scalar_first=false){
    double w, x, y, z;
    if (scalar_first){
        w = quaternion[0];
        x = quaternion[1];
        y = quaternion[2];
        z = quaternion[3];
    }
    else{
        w = quaternion[3];
        x = quaternion[0];
        y = quaternion[1];
        z = quaternion[2];
    }
    std::array<std::array<double, 3>, 3> rotation_matrix = {
        {
            {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
            {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
            {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}
        }
    };
    return rotation_matrix;
}

/**
 * @brief Transform a global pose to local frame relative to reference pose
 * 
 * @param reference_pose Reference pose [x, y, z, qw, qx, qy, qz]
 * @param global_pose Global pose to transform [x, y, z, qw, qx, qy, qz]
 * @return std::vector<double> Local pose [x, y, z, qw, qx, qy, qz]
 */
std::vector<double> transform_global_to_local_frame(
    const std::vector<double>& reference_pose,
    const std::vector<double>& global_pose)
{
    // Extract positions
    double ref_x = reference_pose[0];
    double ref_y = reference_pose[1];
    double ref_z = reference_pose[2];
    
    // Extract reference orientation as yaw
    std::array<double, 4> ref_quat = {
        reference_pose[3], reference_pose[4], 
        reference_pose[5], reference_pose[6]
    };
    double ref_yaw = quaternion_to_yaw(ref_quat, true);
    
    // Extract global position and orientation
    double global_x = global_pose[0];
    double global_y = global_pose[1];
    double global_z = global_pose[2];
    std::array<double, 4> global_quat = {
        global_pose[3], global_pose[4], 
        global_pose[5], global_pose[6]
    };
    double global_yaw = quaternion_to_yaw(global_quat, true);
    
    // Translate global position relative to reference
    double dx = global_x - ref_x;
    double dy = global_y - ref_y;
    
    // Rotate translated position by negative reference yaw
    double cos_ref = std::cos(-ref_yaw);
    double sin_ref = std::sin(-ref_yaw);
    double local_x = dx * cos_ref - dy * sin_ref;
    double local_y = dx * sin_ref + dy * cos_ref;
    
    // Calculate relative orientation (local yaw)
    double local_yaw = global_yaw - ref_yaw;
    
    // Normalize yaw to [-π, π]
    while (local_yaw > M_PI) local_yaw -= 2.0 * M_PI;
    while (local_yaw < -M_PI) local_yaw += 2.0 * M_PI;
    
    // Convert local yaw to quaternion
    std::array<double, 4> local_quat = yaw_to_quaternion(local_yaw, true);
    
    // Return local pose vector [x, y, z, qw, qx, qy, qz]
    return {
        local_x, local_y, global_z - ref_z,
        local_quat[0], local_quat[1], local_quat[2], local_quat[3]
    };
}

/**
 * @brief Calculate distance between quaternions considering object symmetry
 * 
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @param symmetry_rotations Number of symmetric rotations around Z axis
 * @param scalar_first If true, quaternion is [w,x,y,z], if false it's [x,y,z,w]
 * @return double Minimum distance between quaternions considering symmetry
 */
double quaternion_distance_symmetric(
    const std::array<double, 4>& q1,
    const std::array<double, 4>& q2,
    int symmetry_rotations,
    bool scalar_first = true
) {
    // Normalize quaternions and handle scalar position
    auto normalize = [scalar_first](std::array<double, 4> q) {
        double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        std::array<double, 4> normalized = {q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm};
        
        // If scalar-last, convert to scalar-first for internal calculations
        if (!scalar_first) {
            normalized = {normalized[3], normalized[0], normalized[1], normalized[2]};
        }
        return normalized;
    };
    
    auto q1_norm = normalize(q1);
    auto q2_norm = normalize(q2);
    
    // Check all symmetric rotations
    double min_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < symmetry_rotations; ++i) {
        double angle = i * (2 * M_PI / symmetry_rotations);
        
        // Create rotation quaternion around Z axis (in scalar-first format)
        double half_angle = angle * 0.5;
        std::array<double, 4> sym_rot = {
            std::cos(half_angle),  // w
            0,                     // x
            0,                     // y
            std::sin(half_angle)   // z
        };
        
        // Apply symmetric rotation (quaternion multiplication)
        std::array<double, 4> q2_sym = {
            sym_rot[0]*q2_norm[0] - sym_rot[3]*q2_norm[3],  // w
            sym_rot[0]*q2_norm[1] - sym_rot[3]*q2_norm[2],  // x
            sym_rot[0]*q2_norm[2] + sym_rot[3]*q2_norm[1],  // y
            sym_rot[0]*q2_norm[3] + sym_rot[3]*q2_norm[0]   // z
        };
        
        // Calculate distance
        double dot_product = std::abs(
            q1_norm[0]*q2_sym[0] + 
            q1_norm[1]*q2_sym[1] + 
            q1_norm[2]*q2_sym[2] + 
            q1_norm[3]*q2_sym[3]
        );
        dot_product = std::min(1.0, std::max(-1.0, dot_product));
        double dist = 1.0 - dot_product;
        
        min_dist = std::min(min_dist, dist);
    }
    
    return min_dist;
}

/**
 * @brief Simple timer class for profiling code sections
 */
class Timer {
private:
    std::string name;
    std::chrono::high_resolution_clock::time_point start_time;
    std::unordered_map<std::string, double>& timing_map;
    std::unordered_map<std::string, double>& total_timing_map;
    bool active;

public:
    /**
     * @brief Create a new timer and start it
     * 
     * @param name Name of the section being timed
     * @param timing_map Map to store individual timing results
     * @param total_timing_map Map to store cumulative timing results
     */
    Timer(const std::string& name, 
          std::unordered_map<std::string, double>& timing_map,
          std::unordered_map<std::string, double>& total_timing_map)
        : name(name), timing_map(timing_map), total_timing_map(total_timing_map), active(true) {
        start_time = std::chrono::high_resolution_clock::now();
    }

    /**
     * @brief Stop the timer and record elapsed time
     */
    void stop() {
        if (active) {
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            double ms = duration.count() / 1000.0;
            
            timing_map[name] = ms;
            total_timing_map[name] += ms;
            active = false;
        }
    }

    /**
     * @brief Destructor automatically stops the timer if still active
     */
    ~Timer() {
        stop();
    }
};

} // namespace prx 
