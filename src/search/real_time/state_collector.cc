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
			       bool store_exploration_data,
			       ExpansionDelay *expansion_delay,
			       HeuristicError *heuristic_error,
			       SearchEngine const *search_engine)
	: EagerLookaheadSearch(state_registry, store_exploration_data, expansion_delay, heuristic_error, search_engine)
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

AStarCollect::AStarCollect(StateRegistry &state_registry, std::shared_ptr<Evaluator> heuristic, bool store_exploration_data, ExpansionDelay *expansion_delay, HeuristicError *heuristic_error, SearchEngine const *search_engine) :
	EagerCollector(state_registry, store_exploration_data, expansion_delay, heuristic_error, search_engine),
	f_evaluator(std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{heuristic, std::make_shared<g_evaluator::GEvaluator>()})),
	heuristic(heuristic) {}

auto AStarCollect::create_open_list() const -> std::unique_ptr<StateOpenList> {
	auto options = options::Options();
	options.set("evals", std::vector<std::shared_ptr<Evaluator>>{f_evaluator, heuristic});
	options.set("pref_only", false);
	options.set("unsafe_pruning", false);
	return std::make_unique<tiebreaking_open_list::TieBreakingOpenListFactory>(options)->create_state_open_list();
}


}
