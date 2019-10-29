#ifndef REAL_TIME_MAX_TIME_H
#define REAL_TIME_MAX_TIME_H

#include "bound.h"

#include <chrono>

namespace real_time
{

struct MaxTime : Bound
{
	std::chrono::time_point<std::chrono::system_clock> start;
	const std::chrono::duration<int, std::milli> max_ms;
	std::chrono::time_point<std::chrono::system_clock> final_bound;
	int lookahead_part;
	std::chrono::duration<int, std::milli> lookahead_ms;
	std::chrono::time_point<std::chrono::system_clock> lookahead_bound;

	MaxTime(int max_ms);
	virtual ~MaxTime() = default;

	bool lookahead_ok() const final;
	bool learning_ok() const final;
	void initialize(LookaheadSearch const &ls) final;
	void adjust_learning(size_t effort, size_t remaining) final;
	void print_surplus() const final;
};


}

#endif
