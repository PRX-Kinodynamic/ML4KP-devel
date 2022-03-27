#include "prx/gazebo/plugins/execute_plan_plugin.hpp"

namespace prx
{

	execute_plan_plugin_t::execute_plan_plugin_t()
	{
	}

	execute_plan_plugin_t::~execute_plan_plugin_t()
	{

	}

	void execute_plan_plugin_t::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
		std::cout << "Hello from execute_plan_plugin_t" << std::endl; 
    	
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Pendulum plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. For the pendulum, there should only be one joint
      this->joint = _model->GetJoints()[0];

      
      // Create the node
      this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
      this->node->Init(this->model->GetWorld()->Name());

      // // Create a topic name
      std::string topic_name = "~/" + this->model->GetName() + "/execute_plan";

      // // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topic_name,
         &execute_plan_plugin_t::process_message, this);
    }

}