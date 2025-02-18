#include "prx/utilities/defs.hpp"
#include <cmath>

namespace prx {

class PushController {
public:
    PushController(param_loader params, space_point_t control_point) {
        this->params = params;
        this->control_point = control_point;
    }

    virtual ~PushController() = default;
    
    virtual void compute_control(
        const space_point_t current_state,
        const space_point_t goal_state) = 0;

protected:
    param_loader params;
    space_point_t control_point;
};

class SimplePushController : public PushController {
public:
    SimplePushController(param_loader params, space_point_t control_point) : PushController(params, control_point) {
    }
    
    void compute_control(
        const space_point_t current_state,
        const space_point_t goal_state ) override {
        
        // Calculate direction to goal
        double dx = goal_state->at(0) - current_state->at(0);
        double dy = goal_state->at(1) - current_state->at(1);
        double angle = atan2(dy, dx);

        bool allow_collision = params["allow_collision"].as<bool>();
        double scale = params["control_scale"].as<double>();

        double distance = sqrt(dx*dx + dy*dy);
        if (allow_collision && distance < 0.1){
            scale = std::min(scale, distance);
        }
        
        control_point->at(0) = cos(angle) * scale;
        control_point->at(1) = sin(angle) * scale;
    }
}; 

}  // namespace prx
