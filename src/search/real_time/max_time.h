#ifndef REAL_TIME_MAX_TIME_H
#define REAL_TIME_MAX_TIME_H

#include "bound.h"

#include <chrono>

namespace real_time
{

struct MaxTime : Bound
{
	std::chrono::time_point<std::chrono::system_clock> start;
	const std::chrono::milliseconds max_ms;
	std::chrono::time_point<std::chrono::system_clock> final_bound;
	int lookahead_part;
	std::chrono::milliseconds lookahead_ms;
	std::chrono::time_point<std::chrono::system_clock> lookahead_bound;
	LookaheadSearch const *ls;
	std::chrono::milliseconds lookahead_ub; // upper bound estimate on the time of one lookahead iteration

	MaxTime(int max_ms);
	virtual ~MaxTime() = default;

	bool lookahead_ok() final;
	bool learning_ok() const final;
	void initialize(LookaheadSearch const &ls) final;
	void print_surplus() const final;
};


}

#endif
