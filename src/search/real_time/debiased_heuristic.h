#ifndef REAL_TIME_DEBIASED_HEURISTIC_H
#define REAL_TIME_DEBIASED_HEURISTIC_H

#include "heuristic_error.h"
#include "../evaluator.h"
#include "../per_state_information.h"

namespace real_time {

class DebiasedHeuristic : public Evaluator {
	std::shared_ptr<Evaluator> heuristic;
	std::shared_ptr<Evaluator> distance;
	const HeuristicError &heuristic_error;

public:
	DebiasedHeuristic(std::shared_ptr<Evaluator> heuristic, std::shared_ptr<Evaluator> distance, const HeuristicError &heuristic_error);
	~DebiasedHeuristic() override = default;

	auto dead_ends_are_reliable() const -> bool override { return heuristic->dead_ends_are_reliable(); }
	void get_path_dependent_evaluators(std::set<Evaluator *> &evals) override { heuristic->get_path_dependent_evaluators(evals); }

	void notify_initial_state(const GlobalState &initial_state) override { heuristic->notify_initial_state(initial_state); distance->notify_initial_state(initial_state); }
	void notify_state_transition(const GlobalState &parent_state, OperatorID op_id, const GlobalState &state) override { heuristic->notify_state_transition(parent_state, op_id, state); distance->notify_state_transition(parent_state, op_id, state); }

	auto compute_result(EvaluationContext &eval_context) -> EvaluationResult override;

	auto does_cache_estimates() const -> bool override { return false; }
};

}

#endif
