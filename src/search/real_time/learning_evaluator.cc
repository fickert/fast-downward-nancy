#include "learning_evaluator.h"

#include "../evaluation_context.h"

namespace real_time {

LearningEvaluator::LearningEvaluator(std::shared_ptr<Evaluator> base_evaluator) 
	: base_evaluator(base_evaluator),
	  updated_values(NO_VALUE) {}

void LearningEvaluator::update_value(const GlobalState &state, int new_value, bool check_base_evaluator) {
	if (check_base_evaluator) {
		assert(base_evaluator->is_estimate_cached(state));
		new_value = std::max(new_value, base_evaluator->get_cached_estimate(state));
	}
	updated_values[state] = new_value;
}

auto LearningEvaluator::compute_result(EvaluationContext &eval_context) -> EvaluationResult {
	auto &cached_result = updated_values[eval_context.get_state()];
	if (cached_result != NO_VALUE) {
		auto result = EvaluationResult();
		result.set_count_evaluation(false);
		result.set_evaluator_value(cached_result);
		assert(cached_result == EvaluationResult::INFTY || cached_result >= base_evaluator->compute_result(eval_context).get_evaluator_value());
		return result;
	}
	auto result = base_evaluator->compute_result(eval_context);
	cached_result = result.is_infinite() ? EvaluationResult::INFTY : result.get_evaluator_value();
	return result;
}

}
