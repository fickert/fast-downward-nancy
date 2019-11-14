#ifndef REAL_TIME_UTIL_H
#define REAL_TIME_UTIL_H

#include "debiased_heuristic.h"
#include "heuristic_error.h"
#include "../evaluator.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"

#include <memory>
#include <vector>
#include <numeric>

namespace real_time {

inline auto create_f_hat_evaluator(std::shared_ptr<Evaluator> heuristic, std::shared_ptr<Evaluator> distance, HeuristicError &heuristic_error) -> std::shared_ptr<Evaluator> {
	const auto h_hat_evaluator = std::make_shared<DebiasedHeuristic>(heuristic, distance, heuristic_error);
	const auto g_evaluator = std::make_shared<g_evaluator::GEvaluator>();
	return std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{h_hat_evaluator, g_evaluator});
}

template<typename T>
std::pair<size_t, bool> vec_find(std::vector<T> const &v, T const &t)
{
  size_t const s = v.size();
  for (size_t i = 0; i < s; ++i) {
    if (t == v[i]) {
      return std::make_pair(i, true);
    }
  }
  return std::make_pair(0, false);
}

}

#endif
