#ifndef REAL_TIME_DECISION_H
#define REAL_TIME_DECISION_H

#include <vector>

#include "../state_registry.h"
#include "../search_engine.h"
#include "../operator_id.h"
#include "../state_id.h"
#include "DiscreteDistribution.h"

namespace real_time
{

struct Decision
{
	StateRegistry const &state_registry;

	Decision(StateRegistry const &sr);
	virtual ~Decision() = default;

	virtual OperatorID decide(std::vector<StateID> const &frontier, SearchSpace &ss) = 0;
};

}

#endif
