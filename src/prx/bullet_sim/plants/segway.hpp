#ifndef BULLET_NOT_BUILT
#include "prx/bullet_sim/plants/bullet_plant.hpp"

namespace prx
{
    class segway_t : public bullet_plant_t
    {
    public:
        segway_t(const std::string& path);
      
        virtual ~segway_t();

        virtual void initialize(std::shared_ptr<b3RobotSimulatorClientAPI> sim) override;

        virtual void update_from_bullet(const bool save_sim_state) override final;

        virtual void compute_control() override final;

        virtual void reset() override final;

        bool sync_lr_wheels = true;
      
    protected:
        const std::string robot_model_path = models_path + "/segway_440LE/segway_440LE.urdf";  //Note, you will need to change this path

        std::vector<int> wheelJoints;

        double x,y,yaw,dx,dy,dyaw,sid,lf,rf,lr,rr;
        double maxForce = 100;
        double controlMultiplier = 0.5;
 
      
    private:      
        void shared_constructor(const std::string& path,std::vector<double> start_state);
    };
}
PRX_REGISTER_SYSTEM(segway_t, segway)
#endif
