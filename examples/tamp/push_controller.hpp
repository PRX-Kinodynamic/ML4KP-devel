#include "prx/utilities/defs.hpp"

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
        
        // Calculate angle between current position and goal
        double dx = goal_state->at(0) - current_state->at(0);
        double dy = goal_state->at(1) - current_state->at(1);
        double angle = atan2(dy, dx);
        
        // Update our internal control point
        control_point->at(0) = cos(angle);
        control_point->at(1) = sin(angle);
        
        // std::cout << "Control inside: " << control_point->at(0) << ", " << control_point->at(1) << std::endl;
        // Copy to output control;
    }
}; 


}  // namespace prx
