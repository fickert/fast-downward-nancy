#include "lookahead_control.h"

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
	switch (res) {
	case FAILED:        return FAILED;
	case SOLVED:        return SOLVED;
	case IN_PROGRESS:   ls->post(); return IN_PROGRESS;
	case TIMEOUT:       return TIMEOUT;
	}

	__builtin_unreachable();
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
