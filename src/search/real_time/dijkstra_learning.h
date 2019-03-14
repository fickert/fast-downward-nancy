#ifndef REAL_TIME_DIJKSTRA_LEARNING_H
#define REAL_TIME_DIJKSTRA_LEARNING_H

#include "learning_evaluator.h"
#include "../open_list.h"

namespace real_time {

class DijkstraLearning {
	std::shared_ptr<LearningEvaluator> learning_evaluator;
	const StateRegistry &state_registry;

public:
	DijkstraLearning(std::shared_ptr<LearningEvaluator> learning_evaluator, const StateRegistry &state_registry);
	~DijkstraLearning() = default;

	void apply_updates(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors, const std::vector<StateID> &frontier, const std::unordered_set<StateID> &closed) const;
	void apply_updates(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors, const std::vector<StateID> &frontier, std::unordered_set<StateID> &&closed) const;
};

}

#endif
