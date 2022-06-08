#include "prx/planning/planners/prm.hpp"

namespace prx
{
	prm_t::prm_t(const std::string& new_name) : planner_t(new_name)
	{
		metric = nullptr;
		planner_name = new_name;
	}
	prm_t::~prm_t()
	{
		_reset();
	}
	void prm_t::_link_and_setup_spec(planner_specification_t* spec)
	{
		//reset is always called before this
		
		//we now have spaces and necessary functions
	}

	bool prm_t::_preprocess()
	{
        // bulk of the code will go here.
        return true;
	}

	bool prm_t::_link_and_setup_query(planner_query_t* query)
	{
		prm_query = dynamic_cast<prm_query_t*>(query);
		prx_assert(prm_query!=nullptr,"prm received an incorrect query type.");
		
		return true;
	}
	
	void prm_t::_resolve_query(condition_check_t* condition)
	{
		
	}
	void prm_t::_fulfill_query()
	{
		
	}

	void prm_t::_reset()
	{
		//clear the stuff
		tree.purge();
		if(metric!=nullptr)
		{
			delete metric;
			metric = nullptr;
		}

	}
}
