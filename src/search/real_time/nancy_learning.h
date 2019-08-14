#ifndef REAL_TIME_NANCY_LEARNING_H
#define REAL_TIME_NANCY_LEARNING_H

#include "../search_engine.h"
#include "DiscreteDistribution.h"

namespace real_time
{
class NancyLearning
{
  StateRegistry const &state_registry;
  SearchEngine const *search_engine; // need this just so I can call get_adjusted_cost

public:
	NancyLearning(StateRegistry const &state_registry, SearchEngine const *search_engine);

  void apply_updates(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors,
                     const std::vector<StateID> &frontier,
                     const std::unordered_set<StateID> &closed,
                     PerStateInformation<ShiftedDistribution> *beliefs,
                     PerStateInformation<ShiftedDistribution> *post_beliefs,
                     GlobalState const &current_state) const;

  void apply_updates(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors,
                     const std::vector<StateID> &frontier,
                     std::unordered_set<StateID> &&closed,
                     PerStateInformation<ShiftedDistribution> *beliefs,
                     PerStateInformation<ShiftedDistribution> *post_beliefs,
                     GlobalState const &current_state) const;
};

// class NancyLearning
// {
// 	std::shared_ptr<LearningEvaluator> learning_evaluator;
// 	std::shared_ptr<LearningEvaluator> distance_learning_evaluator;
//  PerStateInformation<ShiftedDistribution> *beliefs;
// 	const StateRegistry &state_registry;

// public:
// 	NancyLearning(std::shared_ptr<LearningEvaluator> learning_evaluator, const StateRegistry &state_registry);
// 	NancyLearning(std::shared_ptr<LearningEvaluator> learning_evaluator, std::shared_ptr<LearningEvaluator> distance_learning_evaluator, const StateRegistry &state_registry);
// 	~NancyLearning() = default;

// 	void apply_updates(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors, const std::vector<StateID> &frontier, const std::unordered_set<StateID> &closed) const;
// 	void apply_updates(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors, const std::vector<StateID> &frontier, std::unordered_set<StateID> &&closed) const;
// };
}

#endif
