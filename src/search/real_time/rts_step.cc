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
SearchStatus RealTimeSearch::step() {
	++num_rts_phases;
	sc.initialize_lookahead(current_state);
	TRACKP("doing lookahead");
	const auto status = sc.search();
	TRACKP("finished lookahead");

	const SearchStatistics &current_stats = sc.ls->get_statistics();
	statistics.inc_expanded(current_stats.get_expanded());
	statistics.inc_evaluated_states(current_stats.get_evaluated_states());
	statistics.inc_evaluations(current_stats.get_evaluations());
	statistics.inc_generated(current_stats.get_generated());
	statistics.inc_generated_ops(current_stats.get_generated_ops());
	statistics.inc_reopened(current_stats.get_reopened());

	if (status == FAILED)
		return FAILED;

	if (status == SOLVED) {
		auto overall_plan = Plan();
		// NOTE: the returned plan does not correspond to the agent's actual movements as it removes cycles
		search_space.trace_path(current_state, overall_plan);
		auto plan = sc.ls->get_plan();
		for (auto op_id : plan)
			solution_cost += get_adjusted_cost(task_proxy.get_operators()[op_id]);
		overall_plan.insert(std::end(overall_plan), std::begin(plan), std::end(plan));
		set_plan(overall_plan);
		return SOLVED;
	}

	if (sc.ls->get_frontier().empty())
		return FAILED;

	TRACKP("action selection");
	OperatorID best_tla = sc.select_action();

	TRACKP("learning/backup");
	sc.learn_initial();

	TRACKP("execution");
	const auto parent_node = search_space.get_node(current_state);
	const auto op = task_proxy.get_operators()[best_tla];
	solution_cost += get_adjusted_cost(op);
	current_state = state_registry.get_successor_state(current_state, op);
	auto next_node = search_space.get_node(current_state);
	if (next_node.is_new())
		next_node.open(parent_node, op, get_adjusted_cost(op));

	return IN_PROGRESS;
}
}

#undef BEGINF
#undef ENDF
#undef TRACKP
