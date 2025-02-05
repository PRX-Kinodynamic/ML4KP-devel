#include "prx/utilities/defs.hpp"
#include <cmath>

namespace prx {

class PushController {
public:
    PushController(space_point_t control_point) {
        this->control_point = control_point;
    }

    virtual ~PushController() = default;
    
    virtual void compute_control(
        const space_point_t current_state,
        const space_point_t goal_state) = 0;

protected:
    space_point_t control_point;
};

class SimplePushController : public PushController {
public:
    SimplePushController(space_point_t control_point) : PushController(control_point) {}
    
    void compute_control(
        const space_point_t current_state,
        const space_point_t goal_state ) override {
        
        // Calculate direction to goal
        double dx = goal_state->at(0) - current_state->at(0);
        double dy = goal_state->at(1) - current_state->at(1);
        
        // Normalize the vector and scale for control
        double magnitude = sqrt(dx*dx + dy*dy);
        if (magnitude > 0) {
            control_point->at(0) = dx/magnitude;
            control_point->at(1) = dy/magnitude;
        } else {
            control_point->at(0) = 0;
            control_point->at(1) = 0;
        }
    }
}; 


}  // namespace prx
