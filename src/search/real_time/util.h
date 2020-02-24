#ifndef REAL_TIME_UTIL_H
#define REAL_TIME_UTIL_H

#include "debiased_heuristic.h"
#include "heuristic_error.h"
#include "../evaluator.h"

#include <memory>
#include <vector>

namespace real_time
{

auto create_f_hat_evaluator(std::shared_ptr<Evaluator> heuristic, std::shared_ptr<Evaluator> distance, HeuristicError &heuristic_error) -> std::shared_ptr<Evaluator>;
bool double_eq(double a, double b);

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
