#include "dist_decider.h"

namespace real_time
{

DistributionDecider::DistributionDecider(StateRegistry const &state_registry,
					 TLAs const *tlas,
					 SearchEngine const &engine,
					 StateID dummy_id)
	: Decision(state_registry), tlas(tlas), engine(engine), target_state_id(dummy_id)
{
	assert(tlas);
}

// beliefs from frontier are already backed up to tlas during expansion.
// Now we just have to select the minimum expected among them.
OperatorID DistributionDecider::decide(std::vector<StateID> const &frontier, SearchSpace &search_space)
{
	// only the scalar decider needs the frontier directly
	(void)&frontier;

	// clear path if we expanded the node we had decided to move
	// to.  we need to look at its successors anyway so we just
	// build a new path.
	GlobalState const target_state = state_registry.lookup_state(target_state_id);
	SearchNode const target_node = search_space.get_node(target_state);
	if (target_node.is_closed()) {
		target_path.clear();
	}

	if (target_path.empty()) {
		target_h_hat = std::numeric_limits<double>::infinity();
		target_f_hat = std::numeric_limits<double>::infinity();
	}

	// TODO: maybe replace double == comparisons with something
	// more appropriate

	double min_h = target_h_hat;
	double min_f = target_f_hat;
	size_t min_id = 0;
	bool new_direction = false;
	for (size_t i = 0; i < tlas->size(); ++i) {
		const double cur = tlas->beliefs[i].expected_cost();
		if (std::numeric_limits<double>::infinity() == cur)
			continue;

		const double tla_f_hat = cur + static_cast<double>(tlas->op_costs[i]);
		if (tla_f_hat > min_f) {
			continue;
		} else {
			const double frontier_h_hat = tlas->states[i].second;
			if (tla_f_hat == min_f) {
				if (frontier_h_hat >= min_h) {
					continue;
				} else {
					new_direction = true;
					min_id = i;
					min_h = frontier_h_hat;
				}
			} else {
				assert(tla_f_hat < min_f);
				// tla_f_hat < target_f_hat
				new_direction = true;
				min_id = i;
				min_h = frontier_h_hat;
				min_f = tla_f_hat;
			}
		}
	}

	if (new_direction && !target_path.empty()) {
		OperatorID next_action = target_path.back();
		new_direction = next_action != tlas->ops[min_id];
	}

	if (new_direction) {
		// build new path
		target_path.clear();
		// TODO: could trace only to successor of initial_id to save the pop_back below
		search_space.trace_path_rev_id_to(tlas->current_state->get_id(), tlas->states[min_id].first, target_path);
		assert(!target_path.empty());
		OperatorID next_action = target_path.back();
		assert(std::find(tlas->ops.begin(), tlas->ops.end(), next_action) != tlas->ops.end());
		target_path.pop_back();
		target_h_hat = min_h;
		target_f_hat = min_f;
		target_state_id = tlas->states[min_id].first;
		return next_action;
	} else {
		// follow path
		assert(!target_path.empty());
		OperatorID next_action = target_path.back();
		assert(std::find(tlas->ops.begin(), tlas->ops.end(), next_action) != tlas->ops.end());
		target_path.pop_back();
		if (min_h < target_h_hat)
			target_h_hat = min_h;
		target_f_hat -= engine.get_adjusted_cost(engine.get_operators()[next_action]);
		return next_action;
	}
}



}
