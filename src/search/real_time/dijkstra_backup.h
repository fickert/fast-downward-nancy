#ifndef REAL_TIME_DIJKSTRA_BACKUP_H
#define REAL_TIME_DIJKSTRA_BACKUP_H

#include <queue>

#include "learning.h"
#include "learning_evaluator.h"

namespace real_time
{

struct DijkstraBackup : public Learning
{
	using QueueEntry = std::pair<int, StateID>;
	struct QueueComp
	{
		bool operator()(const QueueEntry &a, const QueueEntry &b) const
		{
			return b.first != EvaluationResult::INFTY && (a.first == EvaluationResult::INFTY || b.first < a.first);
		}
	};

	// passed in at construction time
	std::shared_ptr<LearningEvaluator> learning_evaluator;
	std::shared_ptr<LearningEvaluator> distance_learning_evaluator;
	const bool h_before;

	// created by us during initialize
	std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueComp> learning_queue;

	DijkstraBackup(const StateRegistry &state_registry, SearchEngine const *search_engine, std::shared_ptr<LearningEvaluator> learning_evaluator, std::shared_ptr<LearningEvaluator> distance_learning_evaluator, bool h_before);
	~DijkstraBackup() = default;

	void initialize(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors,
			const std::vector<StateID> &frontier,
			std::unordered_set<StateID> &closed) final;

	void step() final;
	bool done() final;
	size_t effort() final;
	size_t remaining() final;
};

}

#endif
