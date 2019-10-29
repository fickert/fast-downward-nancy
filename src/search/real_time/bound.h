#ifndef REAL_TIME_BOUND_H
#define REAL_TIME_BOUND_H

#include "lookhead_search.h"

namespace real_time
{

struct Bound
{
	virtual ~Bound() = default;
	virtual bool lookahead_ok() const = 0;
	virtual bool learning_ok() const = 0;
	virtual void initialize(LookaheadSearch const &ls) = 0;
	virtual void adjust_learning(size_t effort, size_t remaining) = 0;
	virtual void print_surplus() const = 0;
};

}



#endif
