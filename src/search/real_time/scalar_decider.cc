#include "scalar_decider.h"

// #define TRACKSD

#ifdef TRACKSD
#define BEGINF(X) std::cout << "SC: ENTER: " << X << "\n";
#define ENDF(X) std::cout << "SC: EXIT: " << X << "\n";
#define TRACKP(X) std::cout << "SC: " << X << "\n";
#else
#define BEGINF(X)
#define ENDF(X)
#define TRACKP(X)
#endif


namespace real_time{

ScalarDecider::ScalarDecider(const StateRegistry &state_registry, decltype(evaluator) evaluator)
	: Decision(state_registry),
	  evaluator(evaluator) {}

OperatorID ScalarDecider::decide(const std::vector<StateID> &frontier, SearchSpace &search_space)
{
	assert(!frontier.empty());
	TRACKP("min element");
	const auto best_state_id = std::min_element(std::begin(frontier), std::end(frontier), [this, &search_space](const auto &lhs, const auto &rhs) {
		return evaluator(lhs, search_space) < evaluator(rhs, search_space);
	});
	auto plan = Plan();
	search_space.trace_path_rev(state_registry.lookup_state(*best_state_id), plan);
	assert(!plan.empty());
	TRACKP("Plan empty: " << plan.empty());
	return plan.back();
}

}

#undef TRACKSD
#undef BEGINF
#undef ENDF
#undef TRACKP
