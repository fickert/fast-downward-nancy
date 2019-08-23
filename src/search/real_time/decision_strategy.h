#ifndef REAL_TIME_DECISION_STRATEGY_H
#define REAL_TIME_DECISION_STRATEGY_H

#include "../operator_id.h"
#include "../state_id.h"
#include "../per_state_information.h"
#include "DiscreteDistribution.h"

#include <functional>

class SearchSpace;
class StateRegistry;
class SearchEngine;

namespace real_time {

struct TLAs;

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

class NancyDecisionStrategy
{
  // using Predecessors = std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>>;
  // using Beliefs = PerStateInformation<ShiftedDistribution>;
	// const StateRegistry &state_registry;
  // PerStateInformation<ShiftedDistribution> const *beliefs;
  const TLAs *tlas;
  SearchEngine const &engine; // need this just so I can call get_adjusted_cost
  // Beliefs const *beliefs;

  double target_h_hat;
  double target_f_hat;
  std::vector<OperatorID> target_path;

public:
	NancyDecisionStrategy(TLAs const *tlas,
                        SearchEngine const &engine);
	~NancyDecisionStrategy() = default;
  OperatorID pick_top_level_action(SearchSpace const &search_space);

};

}

#endif
