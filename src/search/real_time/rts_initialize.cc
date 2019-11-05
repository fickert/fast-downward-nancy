#include "real_time_search.h"

#include <iostream>

// #define TRACKRT

#ifdef TRACKRT
#define BEGINF(X) std::cout << "RT: ENTER: " << X << "\n";
#define ENDF(X) std::cout << "RT: EXIT: " << X << "\n";
#define TRACKP(X) std::cout << "RT: " << X << "\n";
#else
#define BEGINF(X)
#define ENDF(X)
#define TRACKP(X)
#endif

namespace real_time
{

void RealTimeSearch::initialize()
{
	BEGINF(__func__);
	assert(heuristic);
	std::cout << "Conducting real-time search" << std::endl;

	auto eval_context = EvaluationContext(current_state, &statistics, false);
	const auto dead_end = eval_context.is_evaluator_value_infinite(heuristic.get());
	statistics.inc_evaluated_states();
	print_initial_evaluator_values(eval_context);

	if (dead_end) {
		std::cout << "Initial state is a dead end, no solution" << std::endl;
		if (heuristic->dead_ends_are_reliable())
			utils::exit_with(utils::ExitCode::SEARCH_UNSOLVABLE);
		else
			utils::exit_with(utils::ExitCode::SEARCH_UNSOLVED_INCOMPLETE);
	}

	auto node = search_space.get_node(current_state);
	node.open_initial();
	ENDF(__func__);
}

}

#undef BEGINF
#undef ENDF
#undef TRACKP
