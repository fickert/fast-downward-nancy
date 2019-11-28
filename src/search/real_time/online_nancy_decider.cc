#include "online_nancy_decider.h"

#include <vector>


namespace real_time
{

OnlineNancyDecider::OnlineNancyDecider(StateRegistry &state_reg, SearchEngine const &engine, GlobalState const &cs, decltype(evaluator) evaluator)
	: Decision(state_reg),
	  engine(engine),
	  cs(&cs),
	  evaluator(evaluator),
	  target_state_id(state_reg.get_initial_state().get_id())
{
}

OperatorID OnlineNancyDecider::decide(const std::vector<StateID> &frontier, SearchSpace &search_space)
{
	assert(!frontier.empty());
	GlobalState const target_state = state_registry.lookup_state(target_state_id);
	SearchNode const target_node = search_space.get_node(target_state);
	if (target_node.is_closed()) {
		target_path.clear();
	}

	if (target_path.empty()) {
		target_h_hat = std::numeric_limits<int>::max();
		target_f_hat = std::numeric_limits<int>::max();
	}

	const auto best_state_id = std::min_element(std::begin(frontier), std::end(frontier), [this, &search_space](const auto &lhs, const auto &rhs) {
												      return evaluator(lhs, search_space) < evaluator(rhs, search_space);
											      });
	auto plan = Plan();
	GlobalState const best_state = state_registry.lookup_state(*best_state_id);
	int frontier_f_hat = evaluator(*best_state_id, search_space);
	int frontier_h_hat = frontier_f_hat - search_space.get_node(best_state).get_g();
	bool new_direction = false;
	int min_h = target_h_hat;
	int min_f = target_f_hat;
	if (frontier_f_hat < target_f_hat) {
		new_direction = true;
		min_f = frontier_f_hat;
		min_h = frontier_h_hat;
	} else if (frontier_f_hat == target_f_hat) {
		if (frontier_h_hat < target_h_hat) {
			new_direction = true;
			min_f = target_f_hat;
			min_h = frontier_h_hat;
		}
	}

	if (new_direction) {
		target_path.clear();
		search_space.trace_path_rev_id_to(cs->get_id(), *best_state_id, target_path);
		assert(!target_path.empty());
		OperatorID next_action = target_path.back();
		// assert(std::find(tlas->ops.begin(), tlas->ops.end(), next_action) != tlas->ops.end());
		target_path.pop_back();
		target_h_hat = min_h;
		target_f_hat = min_f;
		target_state_id = *best_state_id;
		return next_action;
	} else {
		assert(!target_path.empty());
		OperatorID next_action = target_path.back();
		// assert(std::find(tlas->ops.begin(), tlas->ops.end(), next_action) != tlas->ops.end());
		target_path.pop_back();
		if (min_h < target_h_hat)
			target_h_hat = min_h;
		target_f_hat -= engine.get_adjusted_cost(engine.get_operators()[next_action]);
		return next_action;
	}



	assert(!plan.empty());
	return plan.front();
}

}
