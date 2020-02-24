#include "util.h"

#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"

namespace real_time
{

auto create_f_hat_evaluator(std::shared_ptr<Evaluator> heuristic, std::shared_ptr<Evaluator> distance, HeuristicError &heuristic_error) -> std::shared_ptr<Evaluator> {
	const auto h_hat_evaluator = std::make_shared<DebiasedHeuristic>(heuristic, distance, heuristic_error);
	const auto g_evaluator = std::make_shared<g_evaluator::GEvaluator>();
	return std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{h_hat_evaluator, g_evaluator});
}

bool double_eq(double a, double b)
{
	return std::abs(a-b) < std::abs(std::min(a,b)) * std::numeric_limits<double>::epsilon();
}


}
