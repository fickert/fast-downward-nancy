#include "real_time_search.h"

#include <iostream>

namespace real_time
{
SearchStatus RealTimeSearch::step() {
	++num_rts_phases;
	sc.initialize_lookahead(current_state);
	const auto status = sc.search();

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

	OperatorID best_tla = sc.select_action();

	sc.learn_initial();

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
