#ifndef BULLET_NOT_BUILT
#include "prx/bullet_sim/plants/bullet_plant.hpp"

namespace prx
{
    class prius_t : public bullet_plant_t
    {
        public:
            prius_t(const std::string& path);

            virtual ~prius_t();

            virtual void initialize(std::shared_ptr<b3RobotSimulatorClientAPI> sim) override;

            virtual void update_from_bullet(const bool save_sim_state) override final;

            virtual void compute_control() override final;

        protected:

            const std::string robot_model_path = models_path + "/prius/prius.urdf";  //Note, you will need to change this path

            std::vector<int> wheelJoints;
            std::vector<int> steerJoints;

            double x,y,z,r,p,yaw,dx,dy,dz,dr,dp,dyaw,sid,v,w;
            double maxForce = 300;

        private:
            void shared_constructor(const std::string& path,std::vector<double> start_state);
    };
}
PRX_REGISTER_SYSTEM(prius_t, prius)
#endif