#include "state_collector.h"

#include "debiased_heuristic.h"
#include "expansion_delay.h"
#include "heuristic_error.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../open_lists/tiebreaking_open_list.h"
#include "../options/plugin.h"
#include "../tasks/root_task.h"
#include "../task_utils/task_properties.h"
#include "../open_lists/best_first_open_list.h"


namespace real_time
{

void EagerCollector::collect(StateID id)
{
	expanded_states->insert(id);
}

EagerCollector::EagerCollector(StateRegistry &state_registry,
			       int lookahead_bound,
			       bool store_exploration_data,
			       ExpansionDelay *expansion_delay,
			       HeuristicError *heuristic_error,
			       SearchEngine const *search_engine)
	: LookaheadSearch(state_registry, lookahead_bound, store_exploration_data, expansion_delay, heuristic_error, search_engine)
{
	expanded_states = std::make_unique<std::unordered_set<StateID> >();
}

std::unique_ptr<std::unordered_set<StateID> > EagerCollector::get_expanded_states()
{
	assert(expanded_states != nullptr);
	return std::move(expanded_states);
}

void EagerCollector::mark_expanded(SearchNode &node)
{
	statistics->inc_expanded();
	collect(node.get_state_id());
	node.close();
	if (store_exploration_data)
		closed.emplace(node.get_state_id());
	if (expansion_delay)
		expansion_delay->update_expansion_delay(statistics->get_expanded() - open_list_insertion_time[node.get_state_id()]);
}

}
