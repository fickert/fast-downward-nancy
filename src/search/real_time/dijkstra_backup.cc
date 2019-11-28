#include "dijkstra_backup.h"
#include "../evaluation_context.h"

// #define TRACKDB

#ifdef TRACKDB
#define BEGINF(X) std::cout << "DB: ENTER: " << X << "\n";
#define ENDF(X) std::cout << "DB: EXIT: " << X << "\n";
#define TRACKP(X) std::cout << "DB: " << X << "\n";
#else
#define BEGINF(X)
#define ENDF(X)
#define TRACKP(X)
#endif


namespace real_time
{

DijkstraBackup::DijkstraBackup(const StateRegistry &state_registry, SearchEngine const *search_engine, std::shared_ptr<LearningEvaluator> learning_evaluator, std::shared_ptr<LearningEvaluator> distance_learning_evaluator, bool h_before)
	:Learning(state_registry, search_engine), learning_evaluator(learning_evaluator), distance_learning_evaluator(distance_learning_evaluator), h_before(h_before)
{
}

void DijkstraBackup::initialize(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors_,
	const std::vector<StateID> &frontier,
	std::unordered_set<StateID> &closed_)
{
	TRACKP("setting predecessors and closed");
	predecessors = &predecessors_;
	closed = &closed_;
	TRACKP("getting initial effort");
	initial_effort = closed->size();

	if (h_before) {
		TRACKP("computing h before");
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
		for (const auto &state_id : (*closed))
			evaluate(state_id);
	}


	TRACKP("iterating over closed");
	for (const auto &state_id : (*closed))
		learning_evaluator->update_value(state_registry.lookup_state(state_id), EvaluationResult::INFTY);

	TRACKP("iterating over frontier");
	for (const auto &state_id : frontier) {
		auto state = state_registry.lookup_state(state_id);
		assert(learning_evaluator->is_estimate_cached(state));
		learning_queue.emplace(learning_evaluator->get_cached_estimate(state), state_id);
		//closed->erase(state_id);
	}
}


void DijkstraBackup::step()
{
	BEGINF(__func__);
	assert(!done());

 get_entry:
	if (done())
		return;
	auto [h, state_id] = learning_queue.top();
	learning_queue.pop();
	auto state = state_registry.lookup_state(state_id);
	assert(learning_evaluator->is_estimate_cached(state));
	if (learning_evaluator->get_cached_estimate(state) != h)
		goto get_entry;
	if (h == EvaluationResult::INFTY)
		goto get_entry;

	auto cls = closed->find(state_id);
	if (cls != std::end(*closed))
		closed->erase(cls);

	assert(closed->find(state_id) == std::end(*closed));

	auto predecessors_pos = predecessors->find(state_id);
	if (predecessors_pos == std::end(*predecessors))
		return; // state is the root state of the current lookahead search

	for (const auto &[predecessor_id, op] : predecessors_pos->second) {
		cls = closed->find(predecessor_id);
		if (cls == std::end(*closed))
			continue;
		auto predecessor = state_registry.lookup_state(predecessor_id);
		assert(learning_evaluator->is_estimate_cached(predecessor));
		const auto predecessor_h = learning_evaluator->get_cached_estimate(predecessor);
		const auto new_h = h + search_engine->get_adjusted_cost(op);
		if (predecessor_h > new_h) {
			// NOTE: the base evaluator should not need to be checked for consistent heuristics
			learning_evaluator->update_value(predecessor, new_h, true);
			learning_queue.emplace(new_h, predecessor_id);
		}
		if (distance_learning_evaluator) {
			auto eval_context_predecessor = EvaluationContext(predecessor, 0, true, nullptr);
			auto eval_context = EvaluationContext(state, 0, true, nullptr);
			assert(!eval_context_predecessor.is_evaluator_value_infinite(distance_learning_evaluator.get()));
			assert(!eval_context.is_evaluator_value_infinite(distance_learning_evaluator.get()));
			const auto predecessor_d = eval_context_predecessor.get_evaluator_value(distance_learning_evaluator.get());
			const auto new_d = eval_context.get_evaluator_value(distance_learning_evaluator.get()) + 1;
			if (new_d > predecessor_d)
				// NOTE: no need to check base evaluator, as values are only increased
				distance_learning_evaluator->update_value(predecessor, new_d, false);
		}
	}
	ENDF(__func__);
}

bool DijkstraBackup::done()
{
	return learning_queue.empty();
}

size_t DijkstraBackup::effort()
{
	return initial_effort;
}

size_t DijkstraBackup::remaining()
{
	return closed->size();
}


}

#undef BEGINF
#undef ENDF
#undef TRACKP
