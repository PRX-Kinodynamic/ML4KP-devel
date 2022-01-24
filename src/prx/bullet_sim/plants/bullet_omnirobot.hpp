#ifndef BULLET_NOT_BUILT
#include "prx/bullet_sim/plants/bullet_plant.hpp"
#include "prx/bullet_sim/bullet_simulator.hpp"

namespace prx
{
    class bullet_omnirobot_t : public bullet_plant_t
    {
        public:
        
        bullet_omnirobot_t(const std::string& path);
      
        virtual ~bullet_omnirobot_t();

        virtual void initialize(std::shared_ptr<b3RobotSimulatorClientAPI> sim) override;

        virtual void update_from_bullet(const bool save_sim_state) override;
        
        virtual void compute_control() override final;

        protected:
        const std::string robot_model_path = models_path + "/RUmnibot/RUmnibot.urdf";

        std::vector<int> wheelJoints;// = {2,3,4,5};

        double x,y,z,r,p,yaw,dx,dy,dz,dr,dp,dyaw,sid;
        double w1, w2, w3, w4;
        double maxForce = 100;
        double controlMultiplier = 0.5;
 
      
        private:      
        void shared_constructor(const std::string& path,std::vector<double> start_state);
    };
}
PRX_REGISTER_SYSTEM(bullet_omnirobot_t, bullet_omnirobot)

#endif
