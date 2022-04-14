#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace prx
{	

	class plant_gz_t
	{
		public:

			plant_gz_t() = default;
			virtual ~plant_gz_t() = default;

			void set_model_ptr(const gazebo::physics::ModelPtr& _m_ptr)
			{
				m_ptr = _m_ptr;
			}

			virtual void copy_to_model_ptr() const = 0;

			virtual void copy_from_model_ptr() = 0;

			virtual void reset_system() = 0;

			void update()
			{
				m_ptr -> Update();
			}

			void set_reset_state(const std::vector<double>& _rv)
			{
				reset_vec.assign(_rv.begin(), _rv.end());
			}

		protected:

			gazebo::physics::ModelPtr m_ptr;
			std::vector<double> reset_vec;

	};

}

#define PRX_REGISTER_SYSTEM_GZ(SYSTEM_CLASS, SYSTEM_NAME) \
namespace prx { namespace factory_registration \
{ \
	static auto FN_##SYSTEM_NAME##_GENERATOR =  [](std::string path) \
	{ \
		return std::shared_ptr<plant_gz_wrapper_t>(new plant_gz_wrapper_t(std::shared_ptr<SYSTEM_CLASS>(new SYSTEM_CLASS(path)))); \
	}; \
	const bool VAR_##SYSTEM_NAME##_REGISTRED = system_factory_t::get().register_system(#SYSTEM_NAME, FN_##SYSTEM_NAME##_GENERATOR); \
} }