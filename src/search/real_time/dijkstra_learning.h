#ifndef REAL_TIME_DIJKSTRA_LEARNING_H
#define REAL_TIME_DIJKSTRA_LEARNING_H

#include "../search_engine.h"
#include "learning_evaluator.h"
#include "../open_list.h"

namespace real_time {

class DijkstraLearning {
	std::shared_ptr<LearningEvaluator> learning_evaluator;
	std::shared_ptr<LearningEvaluator> distance_learning_evaluator;
	StateRegistry const &state_registry;
	SearchEngine const *search_engine; // need this just so I can call get_adjusted_cost

public:
	DijkstraLearning(std::shared_ptr<LearningEvaluator> learning_evaluator, const StateRegistry &state_registry, SearchEngine const *search_engine);
	DijkstraLearning(std::shared_ptr<LearningEvaluator> learning_evaluator, std::shared_ptr<LearningEvaluator> distance_learning_evaluator, const StateRegistry &state_registry, SearchEngine const *search_engine);
	~DijkstraLearning() = default;

	void apply_updates(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors, const std::vector<StateID> &frontier, const std::unordered_set<StateID> &closed, bool evaluate_heuristic) const;
	void apply_updates(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors, const std::vector<StateID> &frontier, std::unordered_set<StateID> &&closed, bool evaluate_heuristic) const;
};

}

#endif
