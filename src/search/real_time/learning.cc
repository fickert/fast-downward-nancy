#include "learning.h"

namespace real_time
{

Learning::Learning(StateRegistry const &state_registry, SearchEngine const *search_engine)
	: state_registry(state_registry),
	  search_engine(search_engine),
	  predecessors(nullptr),
	  closed(nullptr)
{

}

}
