#include "max_time.h"

#include <cmath>

// #define TRACKSC

#ifdef TRACKSC
#include <iostream>
#define BEGINF(X) std::cout << "MT: ENTER: " << X << "\n";
#define ENDF(X) std::cout << "MT: EXIT: " << X << "\n";
#define TRACKP(X) std::cout << "MT: " << X << "\n";
#else
#define BEGINF(X)
#define ENDF(X)
#define TRACKP(X)
#endif

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
	TRACKP("lookahead upper bound estimate: " << lookahead_ub.count());
	return (std::chrono::system_clock::now() + lookahead_ub) < lookahead_bound;
}

bool MaxTime::learning_ok() const
{
	return std::chrono::system_clock::now() < final_bound;
}

void MaxTime::initialize(LookaheadSearch const &ls)
{
	BEGINF(__func__);
	this->ls = &ls;
	start = final_bound;
	final_bound = start + max_ms;
	lookahead_bound = start + lookahead_ms;
	ENDF(__func__);
}

void MaxTime::print_surplus() const
{
	// TRACKP("Time left: " << (final_bound > std::chrono::system_clock::now()));
}
}
