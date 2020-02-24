#ifndef REAL_TIME_SCALAR_DECISION_H
#define REAL_TIME_SCALAR_DECISION_H

#include <functional>
#include "decision.h"

namespace real_time
{

struct ScalarDecider : public Decision
{
	std::function<int(StateID, SearchSpace &)> evaluator;

	ScalarDecider(const StateRegistry &state_registry, decltype(evaluator) evaluator);
	virtual ~ScalarDecider() = default;

	OperatorID decide(std::vector<StateID> const &frontier, SearchSpace &ss) final;
};

}

#endif
