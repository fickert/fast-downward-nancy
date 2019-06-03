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


ProbabilisticDecisionStrategy::ProbabilisticDecisionStrategy(const StateRegistry &state_registry, PerStateInformation<ShiftedDistribution> const *beliefs)
  :state_registry(state_registry),
   beliefs(beliefs)
{
}

bool ProbabilisticDecisionStrategy::state_cheaper(const StateID &a, const StateID &b) const
{
  return (*beliefs)[state_registry.lookup_state(a)].expected_cost()
       < (*beliefs)[state_registry.lookup_state(b)].expected_cost();
}

auto ProbabilisticDecisionStrategy::get_top_level_action(const std::vector<StateID> &frontier, SearchSpace &search_space) const -> OperatorID
{
	assert(!frontier.empty());
	const auto best_state_id = std::min_element(std::begin(frontier), std::end(frontier), [this,&search_space](const auto &a, const auto &b){return this->state_cheaper(a,b);});
  auto plan = Plan();
	search_space.trace_path(state_registry.lookup_state(*best_state_id), plan);
  assert(!plan.empty());
  return plan.front();
}

}
