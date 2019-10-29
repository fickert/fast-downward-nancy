#ifndef REAL_TIME_NANCY_BACKUP_H
#define REAL_TIME_NANCY_BACKUP_H

#include <queue>

#include "DiscreteDistribution.h"
#include "learning.h"

namespace real_time
{

struct NancyBackup : public Learning
{
	using QueueEntry = std::pair<ShiftedDistribution, StateID>;

	struct QueueComp
	{
		bool operator()(const QueueEntry &a, const QueueEntry &b) const
		{
			return a.first.expected_cost() >  b.first.expected_cost();
		}
	};

	// passed in at construction time
	using Beliefs = PerStateInformation<ShiftedDistribution>;
	Beliefs *beliefs;
	Beliefs *post_beliefs;

	// created by us during initialize
	std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueComp> learning_queue;

	NancyBackup(StateRegistry const &state_registry,
                SearchEngine const *search_engine,
                Beliefs *beliefs,
                Beliefs *post_beliefs);

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
