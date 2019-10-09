#ifndef REAL_TIME_STATE_COLLECTOR
#define REAL_TIME_STATE_COLLECTOR

#include <unordered_set>
#include "lookhead_search.h"
#include "../state_id.h"

namespace real_time
{

class EagerCollector : public LookaheadSearch
{
private:
	std::unique_ptr<std::unordered_set<StateID> > expanded_states;
	void mark_expanded(SearchNode &node) final;
	void collect(StateID id);
public:
	std::unique_ptr<std::unordered_set<StateID> > get_expanded_states() final;
	EagerCollector(StateRegistry &state_registry,
		       int lookahead_bound,
		       bool store_exploration_data,
		       ExpansionDelay *expansion_delay,
		       HeuristicError *heuristic_error,
		       SearchEngine const *search_engine);
	~EagerCollector() {}
};

}


#endif
