#ifndef REAL_TIME_ONLINE_NANCY_DECISION_H
#define REAL_TIME_ONLINE_NANCY_DECISION_H

#include "decision.h"
#include <vector>
#include "../state_id.h"
#include "../global_state.h"

namespace real_time
{

class OnlineNancyDecider : public Decision
{
	const SearchEngine &engine;
	GlobalState const *cs;
	std::function<int(StateID, SearchSpace &)> evaluator;
	std::vector<OperatorID> target_path;
	int target_f_hat;
	int target_h_hat;
	StateID target_state_id;

public:
	OnlineNancyDecider(StateRegistry &state_registry, SearchEngine const &engine, GlobalState const &cs, decltype(evaluator) evaluator);
	~OnlineNancyDecider() override = default;

	OperatorID decide(std::vector<StateID> const &frontier, SearchSpace &ss) override;
};
}

#endif
