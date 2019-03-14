#include "decision_strategy.h"

#include "../plan_manager.h"
#include "../search_space.h"

#include <algorithm>

namespace real_time {

ScalarDecisionStrategy::ScalarDecisionStrategy(const StateRegistry &state_registry, decltype(evaluator) evaluator) 
	: state_registry(state_registry),
	  evaluator(evaluator) {}

auto ScalarDecisionStrategy::get_top_level_action(const std::vector<StateID> &frontier, SearchSpace &search_space) const -> OperatorID {
	assert(!frontier.empty());
	const auto best_state_id = std::min_element(std::begin(frontier), std::end(frontier), [this, &search_space](const auto &lhs, const auto &rhs) {
		return evaluator(lhs, search_space) < evaluator(rhs, search_space);
	});
	auto plan = Plan();
	search_space.trace_path(state_registry.lookup_state(*best_state_id), plan);
	assert(!plan.empty());
	return plan.front();
}

}
