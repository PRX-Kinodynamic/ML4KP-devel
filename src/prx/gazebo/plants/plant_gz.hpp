#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace prx
{	
	// template <class PC> 
	// class plant_gz_t;

	// template <typename PC>
	// using plant_gz_ptr_t = std::shared_ptr<plant_gz_t<PC>>;
	// using PC = system_t;
	// template <class PC> // Plant Class
	// class plant_gz_t : public PC
	// class plant_gz_t : public virtual PC
	class plant_gz_t
	{
		// static_assert(std::is_base_of<system_t, PC>::value, "plant_gz_t<PC>, PC must derive from plant_t");
		public:

			plant_gz_t() = default;
			// plant_gz_t(const std::string& path) : system_t(path) {};
			virtual ~plant_gz_t() = default;

			void set_model_ptr(const gazebo::physics::ModelPtr& _m_ptr)
			{
				m_ptr = _m_ptr;
			}

			virtual void copy_to_model_ptr() const = 0;

			virtual void copy_from_model_ptr() = 0;

			void update()
			{
				m_ptr -> Update();
			}

			// virtual void update_configuration() override
			// {}

		protected:

			gazebo::physics::ModelPtr m_ptr;
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