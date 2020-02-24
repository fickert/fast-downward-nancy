#include "real_time_search.h"

#include <iostream>

namespace real_time
{

void RealTimeSearch::initialize()
{
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
}

}
