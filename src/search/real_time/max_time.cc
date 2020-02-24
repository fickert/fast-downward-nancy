#include "max_time.h"

#include <cmath>

namespace real_time
{

MaxTime::MaxTime(int ms)
	: start(std::chrono::system_clock::now()),
	  max_ms(ms),
	  final_bound(start + max_ms),
	  lookahead_part(95),
	  lookahead_ms(ms * lookahead_part / 100),
	  lookahead_bound(start + lookahead_ms),
	  ls(nullptr),
	  lookahead_ub(1) //assume lookahead always takes at least one ms
{}

bool MaxTime::lookahead_ok()
{
	lookahead_ub = std::max(lookahead_ub, ls->get_duration());
	return (std::chrono::system_clock::now() + lookahead_ub) < lookahead_bound;
}

bool MaxTime::learning_ok() const
{
	return std::chrono::system_clock::now() < final_bound;
}

void MaxTime::initialize(LookaheadSearch const &ls)
{
	this->ls = &ls;
	start = final_bound;
	final_bound = start + max_ms;
	lookahead_bound = start + lookahead_ms;
}

void MaxTime::print_surplus() const
{
}
}
