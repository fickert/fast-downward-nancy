#include "debiased_heuristic.h"

#include "../evaluation_context.h"

namespace real_time {

DebiasedHeuristic::DebiasedHeuristic(std::shared_ptr<Evaluator> heuristic, std::shared_ptr<Evaluator> distance, const HeuristicError &heuristic_error)
	: heuristic(heuristic),
	  distance(distance),
	  heuristic_error(heuristic_error) {}

auto DebiasedHeuristic::compute_result(EvaluationContext &eval_context) -> EvaluationResult {
	auto result = EvaluationResult();
	if (eval_context.is_evaluator_value_infinite(heuristic.get()) || eval_context.is_evaluator_value_infinite(distance.get())) {
		result.set_evaluator_value(EvaluationResult::INFTY);
	} else {
		const auto h = eval_context.get_evaluator_value(heuristic.get());
		const auto d = eval_context.get_evaluator_value(distance.get());
		result.set_evaluator_value(std::lround(h + d / (1. - heuristic_error.get_average_distance_error()) * heuristic_error.get_average_heuristic_error()));
	}
	return result;
}

}
