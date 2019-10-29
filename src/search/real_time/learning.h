#ifndef REAL_TIME_LEARNING_H
#define REAL_TIME_LEARNING_H

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../state_id.h"
#include "../search_engine.h"
#include "../state_registry.h"
#include "../task_proxy.h"

namespace real_time
{

struct Learning
{
	// passed in during construction
	StateRegistry const &state_registry;
	SearchEngine const *search_engine; // need this just so I can call get_adjusted_cost
	size_t initial_effort;

	// passed in each iteration during initialize
	std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> const *predecessors;
	std::unordered_set<StateID> *closed; // consumed during learning

	Learning(StateRegistry const &state_registry,
		 SearchEngine const *search_engine);
	virtual ~Learning() = default;

	virtual void initialize(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors,
			const std::vector<StateID> &frontier,
			std::unordered_set<StateID> &closed) = 0;

	virtual void step() = 0;
	virtual bool done() = 0;
	virtual size_t effort() = 0;
	virtual size_t remaining() = 0;
};

}



#endif
