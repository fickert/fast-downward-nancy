#ifndef REAL_TIME_DECISION_STRATEGY_H
#define REAL_TIME_DECISION_STRATEGY_H

#include "../operator_id.h"
#include "../state_id.h"
#include "../per_state_information.h"
#include "DiscreteDistribution.h"

#include <functional>

class SearchSpace;
class StateRegistry;

namespace real_time {

class DecisionStrategy {
public:
	DecisionStrategy() = default;
	virtual ~DecisionStrategy() = default;

	virtual auto get_top_level_action(const std::vector<StateID> &frontier, SearchSpace &search_space) const -> OperatorID = 0;
};

class ScalarDecisionStrategy : public DecisionStrategy {
	const StateRegistry &state_registry;
	std::function<int(StateID, SearchSpace &)> evaluator;

public:
	ScalarDecisionStrategy(const StateRegistry &state_registry, decltype(evaluator) evaluator);
	~ScalarDecisionStrategy() override = default;

	auto get_top_level_action(const std::vector<StateID> &frontier, SearchSpace &search_space) const -> OperatorID override;
};

class ProbabilisticDecisionStrategy : public DecisionStrategy
{
	const StateRegistry &state_registry;
  PerStateInformation<ShiftedDistribution> const *beliefs;

public:
	ProbabilisticDecisionStrategy(const StateRegistry &state_registry, PerStateInformation<ShiftedDistribution> const *beliefs);
	~ProbabilisticDecisionStrategy() override = default;
  auto get_top_level_action(const std::vector<StateID> &frontier, SearchSpace &search_space) const -> OperatorID override;

  virtual bool state_cheaper(const StateID &a, const StateID &b) const;
};

}

#endif
