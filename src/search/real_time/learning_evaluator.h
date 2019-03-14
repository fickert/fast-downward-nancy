#ifndef REAL_TIME_LEARNING_EVALUATOR_H
#define REAL_TIME_LEARNING_EVALUATOR_H

#include "../evaluator.h"
#include "../per_state_information.h"

namespace real_time {

class LearningEvaluator : public Evaluator {
	std::shared_ptr<Evaluator> base_evaluator;
	PerStateInformation<int> updated_values;

public:
	static constexpr auto NO_VALUE = -2;

	explicit LearningEvaluator(std::shared_ptr<Evaluator> base_evaluator);
	~LearningEvaluator() override = default;

	void update_value(const GlobalState &state, int new_value, bool check_base_evaluator = false);

	auto dead_ends_are_reliable() const -> bool override { return base_evaluator->dead_ends_are_reliable(); }
	void get_path_dependent_evaluators(std::set<Evaluator *> &evals) override { base_evaluator->get_path_dependent_evaluators(evals); }

	void notify_initial_state(const GlobalState &initial_state) override { base_evaluator->notify_initial_state(initial_state); }
	void notify_state_transition(const GlobalState &parent_state, OperatorID op_id, const GlobalState &state) override { base_evaluator->notify_state_transition(parent_state, op_id, state); }

	auto compute_result(EvaluationContext &eval_context) -> EvaluationResult override;

	auto does_cache_estimates() const -> bool override { return base_evaluator->does_cache_estimates(); }
	auto is_estimate_cached(const GlobalState &state) const -> bool override { return updated_values[state] != NO_VALUE || base_evaluator->is_estimate_cached(state); }
	auto get_cached_estimate(const GlobalState &state) const -> int override { return updated_values[state] != NO_VALUE ? updated_values[state] : base_evaluator->get_cached_estimate(state); }
};

}

#endif
