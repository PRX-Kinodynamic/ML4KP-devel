#ifndef BULLET_NOT_BUILT
#include "prx/bullet_sim/plants/bullet_plant.hpp"

namespace prx
{
    class racecar_t : public bullet_plant_t
    {
        public:
        racecar_t(const std::string& path);

        virtual ~racecar_t();

        virtual void initialize(std::shared_ptr<b3RobotSimulatorClientAPI> sim) override;

        virtual void update_from_bullet(const bool save_sim_state) override final;

		virtual void compute_control() override final;

        virtual int get_state_id() override;
        
        protected:

        const std::string robot_model_path = bullet_path + "/data/racecar/racecar_differential.urdf";

        double x,y,theta,sid,fwd,steer;

        std::vector<int> steeringLinks = {0,2};
        double maxForce = 20;
        int nMotors = 2;
        std::vector<int> motorizedWheels = {8,15};
        double speedMultiplier = 20;
        double steeringMultiplier = 0.5;
    };
}
#endif
