#include "heuristic_error.h"

#include "../evaluation_context.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"

namespace real_time {

HeuristicError::HeuristicError(const StateRegistry &state_registry, std::shared_ptr<Evaluator> heuristic, std::shared_ptr<Evaluator> distance)
	: state_registry(state_registry),
	  f_evaluator(std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{heuristic, std::make_shared<g_evaluator::GEvaluator>()})),
	  heuristic(heuristic),
	  distance(distance),
	  average_heuristic_error(0),
	  average_distance_error(0),
	  count(0),
	  current_state_id(StateID::no_state),
	  best_successor_id(StateID::no_state),
	  best_successor_op_cost(0),
	  best_successor_value(-1) {}

void HeuristicError::set_expanding_state(const GlobalState &state) {
	current_state_id = state.get_id();
	best_successor_value = -1;
}

void HeuristicError::add_successor(const SearchNode &successor_node, int op_cost) {
	assert(successor_node.is_open() || successor_node.is_closed());
	auto eval_context = EvaluationContext(successor_node.get_state(), successor_node.get_g(), true, nullptr);
	assert(!eval_context.is_evaluator_value_infinite(f_evaluator.get()));
	const auto f_value = eval_context.get_evaluator_value(f_evaluator.get());
	if (best_successor_value == -1 || f_value < best_successor_value) {
		best_successor_value = f_value;
		best_successor_id = successor_node.get_state_id();
		best_successor_op_cost = op_cost;
	}
}

void HeuristicError::update_error() {
	if (best_successor_value != -1) {
		++count;
		const auto state = state_registry.lookup_state(current_state_id);
		const auto successor_state = state_registry.lookup_state(best_successor_id);
		assert(heuristic->is_estimate_cached(state));
		assert(heuristic->is_estimate_cached(successor_state));
		const auto heuristic_error = std::max(0, heuristic->get_cached_estimate(successor_state) + best_successor_op_cost - heuristic->get_cached_estimate(state));
		average_heuristic_error += (heuristic_error - average_heuristic_error) / count;
		if (distance) {
			auto eval_context = EvaluationContext(state, 0, true, nullptr);
			auto eval_context_successor = EvaluationContext(successor_state, 0, true, nullptr);
			assert(!eval_context.is_evaluator_value_infinite(distance.get()));
			assert(!eval_context_successor.is_evaluator_value_infinite(distance.get()));
			const auto distance_error = std::min(1, std::max(0, eval_context_successor.get_evaluator_value(distance.get()) + 1 - eval_context.get_evaluator_value(distance.get())));
			average_distance_error += (distance_error - average_distance_error) / count;
		}
	}
}

}
