#ifndef REAL_TIME_MAX_EXPANSIONS_H
#define REAL_TIME_MAX_EXPANSIONS_H

#include "bound.h"
#include "../search_statistics.h"

namespace real_time
{

struct MaxExpansions : Bound
{
	SearchStatistics const *stats;
	int bound;

	MaxExpansions(int b);
	virtual ~MaxExpansions() = default;

	bool lookahead_ok() final;
	bool learning_ok() const final;
	void initialize(LookaheadSearch const &ls) final;
	void adjust_learning(size_t effort, size_t remaining) final;
	void print_surplus() const final;
};


}

#endif
