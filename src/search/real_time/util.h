#ifndef REAL_TIME_UTIL_H
#define REAL_TIME_UTIL_H

#include "debiased_heuristic.h"
#include "heuristic_error.h"
#include "../evaluator.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"

#include <memory>

namespace real_time {

inline auto create_f_hat_evaluator(std::shared_ptr<Evaluator> heuristic, std::shared_ptr<Evaluator> distance, HeuristicError &heuristic_error) -> std::shared_ptr<Evaluator> {
	const auto h_hat_evaluator = std::make_shared<DebiasedHeuristic>(heuristic, distance, heuristic_error);
	const auto g_evaluator = std::make_shared<g_evaluator::GEvaluator>();
	return std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{h_hat_evaluator, g_evaluator});
}

}

#endif
