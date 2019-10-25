#include "lookahead_control.h"

#include <numeric>   // accumulate
#include <algorithm> // sort

// #define TRACKLC

#ifdef TRACKLC
#define BEGINF(X) std::cout << "LC: ENTER: " << X << "\n";
#define ENDF(X) std::cout << "LC: EXIT: " << X << "\n";
#define TRACKP(X) std::cout << "LC: " << X << "\n";
#else
#define BEGINF(X)
#define ENDF(X)
#define TRACKP(X)
#endif


namespace real_time
{

LookaheadControl::LookaheadControl() : ls(nullptr), lb(nullptr) {}
LookaheadControl::~LookaheadControl() {}

void LookaheadControl::initialize(GlobalState const &s)
{
	BEGINF(__func__);
	ls->initialize(s);
	assert(lb != nullptr);
	lb->initialize(*ls);
	ENDF(__func__)
}

SearchStatus LookaheadControl::search()
{
	BEGINF(__func__);
	SearchStatus res = IN_PROGRESS;
	while (lb->ok() && res == IN_PROGRESS) {
		res = ls->step();
	}
	expansions.push_back(ls->get_statistics().get_expanded());
	switch (res) {
	case FAILED:        return FAILED;
	case SOLVED:        return SOLVED;
	case IN_PROGRESS:   ls->post(); return IN_PROGRESS;
	case TIMEOUT:       return TIMEOUT;
	}

	__builtin_unreachable();
}

void LookaheadControl::print_statistics() const
{
	ls->print_statistics();

	// hack since I need to sort the vector and this is a const
	// method.  this is fine though since the statistics are only
	// printed once in the end, so moving the vector doesn't
	// matter anymore.
	auto v = std::move(expansions);
	std::sort(v.begin(), v.end());
	size_t s = v.size();
	assert(s > 0);
	int avg = std::accumulate(v.begin(), v.end(), 0) / s;
	int min = v[0];
	int max = v[s-1];
	int med = ((s & 1) == 0) ? (v[(s/2)-1] + v[s/2]) / 2 : v[s/2];

	std::cout << "Average number of expansions: " << avg << "\n"
		  << "Minimum number of expansions: " << min << "\n"
		  << "Maximum number of expansions: " << max << "\n"
		  << "Median expansions: " << med << "\n";
}

ExpansionBound::ExpansionBound(int b): stats(nullptr), bound(b) {}

bool ExpansionBound::ok() const
{
	assert(stats != nullptr);
	return stats->get_expanded() < bound;
}

void ExpansionBound::initialize(LookaheadSearch const &ls)
{
	stats = &ls.get_statistics();
	assert(stats->get_expanded() == 1);
}

RtTimer::RtTimer() {}
RtTimer::~RtTimer() {}

// uint64 rdtsc() {
// 	uint32_t hi, lo;
// 	__asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
// 	return ((uint64_t) hi << 32 | lo;
// }

std::chrono::duration<int, std::milli> RtTimer::diff() const
{
	auto d = (std::chrono::system_clock::now() - start);
	return std::chrono::duration_cast<std::chrono::milliseconds>(d);
}

void RtTimer::reset()
{
	BEGINF(__func__);
	start = std::chrono::system_clock::now();
	ENDF(__func__);
}

TimeBound::TimeBound(int ms) : max_ms(std::chrono::duration<int, std::milli>(ms)) {BEGINF(__func__); ENDF(__func__);}

bool TimeBound::ok() const
{
	// BEGINF(__func__);
	// auto d = timer.diff();
	// if (d >= max_ms) {
	// 	ENDF(__func__);
	// 	std::cout << "stopped lookahead after " << d.count() << "\n";
	// 	return false;
	// } else {
	// 	ENDF(__func__);
	// 	return true;
	// }
	return timer.diff() < max_ms;
}

void TimeBound::initialize(LookaheadSearch const &ls)
{
	BEGINF(__func__);
	(void)&ls;
	timer.reset();
	ENDF(__func__);
}


}

#undef BEGINF
#undef ENDF
#undef TRACKP
