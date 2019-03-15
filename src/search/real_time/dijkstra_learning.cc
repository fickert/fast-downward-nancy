#include "dijkstra_learning.h"

#include <queue>

namespace real_time {

DijkstraLearning::DijkstraLearning(std::shared_ptr<LearningEvaluator> learning_evaluator, const StateRegistry &state_registry)
	: learning_evaluator(learning_evaluator), distance_learning_evaluator(nullptr), state_registry(state_registry) {}

DijkstraLearning::DijkstraLearning(std::shared_ptr<LearningEvaluator> learning_evaluator, std::shared_ptr<LearningEvaluator> distance_learning_evaluator, const StateRegistry &state_registry)
	: learning_evaluator(learning_evaluator), distance_learning_evaluator(distance_learning_evaluator), state_registry(state_registry) {}

void DijkstraLearning::apply_updates(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors, const std::vector<StateID> &frontier, const std::unordered_set<StateID> &closed, bool evaluate_heuristic) const {
	auto closed_copy = closed;
	apply_updates(predecessors, frontier, std::move(closed_copy), evaluate_heuristic);
}

void DijkstraLearning::apply_updates(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors, const std::vector<StateID> &frontier, std::unordered_set<StateID> &&closed, bool evaluate_heuristic) const {
	if (evaluate_heuristic) {
		// ensure the heuristic values are cached for all states
		const auto evaluate = [this](const auto &state_id) {
			auto state = state_registry.lookup_state(state_id);
			auto eval_context = EvaluationContext(state);
			learning_evaluator->compute_result(eval_context);
			if (distance_learning_evaluator)
				distance_learning_evaluator->compute_result(eval_context);
		};
		for (const auto &state_id : frontier)
			evaluate(state_id);
		for (const auto &state_id : closed)
			evaluate(state_id);
	}

	for (const auto &state_id : closed)
		learning_evaluator->update_value(state_registry.lookup_state(state_id), EvaluationResult::INFTY);

	auto compare_h = [](const auto &lhs, const auto &rhs) {
		return lhs != EvaluationResult::INFTY && (rhs == EvaluationResult::INFTY || lhs < rhs);
	};
	auto learning_queue_compare = [compare_h](const auto &lhs, const auto &rhs) {
		return compare_h(rhs.first, lhs.first);
	};
	using LearningQueueType = std::pair<int, StateID>;
	auto learning_queue = std::priority_queue<LearningQueueType, std::vector<LearningQueueType>, decltype(learning_queue_compare)>(learning_queue_compare);

	assert(!frontier.empty());
	for (const auto &state_id : frontier) {
		auto state = state_registry.lookup_state(state_id);
		assert(learning_evaluator->is_estimate_cached(state));
		learning_queue.emplace(learning_evaluator->get_cached_estimate(state), state_id);
		closed.erase(state_id);
	}

	while (!learning_queue.empty()) {
		auto [h, state_id] = learning_queue.top();
		learning_queue.pop();
		auto state = state_registry.lookup_state(state_id);
		assert(learning_evaluator->is_estimate_cached(state));
		if (learning_evaluator->get_cached_estimate(state) != h)
			continue;
		if (h == EvaluationResult::INFTY)
			continue;
		assert(closed.find(state_id) == std::end(closed));
		auto predecessors_pos = predecessors.find(state_id);
		if (predecessors_pos == std::end(predecessors))
			// state is the root state of the current lookahead search
			continue;
		for (const auto &[predecessor_id, op] : predecessors_pos->second) {
			auto predecessor = state_registry.lookup_state(predecessor_id);
			auto closed_it = closed.find(predecessor_id);
			if (closed_it != std::end(closed)) {
				assert(learning_evaluator->is_estimate_cached(predecessor));
				const auto predecessor_h = learning_evaluator->get_cached_estimate(predecessor);
				const auto new_h = h + op.get_cost();
				if (predecessor_h > new_h) {
					closed.erase(closed_it);
					// NOTE: the base evaluator should not need to be checked for consistent heuristics
					learning_evaluator->update_value(predecessor, new_h, true);
					learning_queue.emplace(new_h, predecessor_id);
				}
				if (distance_learning_evaluator) {
					assert(distance_learning_evaluator->is_estimate_cached(state));
					assert(distance_learning_evaluator->is_estimate_cached(predecessor));
					const auto predecessor_d = distance_learning_evaluator->get_cached_estimate(predecessor);
					const auto new_d = distance_learning_evaluator->get_cached_estimate(state) + 1;
					if (new_d > predecessor_d)
						// NOTE: no need to check base evaluator, as values are only increased
						distance_learning_evaluator->update_value(predecessor, new_d, false);
				}
			}
		}
	}
}

}
