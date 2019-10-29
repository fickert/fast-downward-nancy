#include "scalar_decider.h"

namespace real_time{

ScalarDecider::ScalarDecider(const StateRegistry &state_registry, decltype(evaluator) evaluator)
	: Decision(state_registry),
	  evaluator(evaluator) {}

OperatorID ScalarDecider::decide(const std::vector<StateID> &frontier, SearchSpace &search_space)
{
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
