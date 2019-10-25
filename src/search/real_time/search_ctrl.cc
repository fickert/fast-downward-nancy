#include "search_ctrl.h"

#include <numeric>   // accumulate
#include <algorithm> // sort

// #define TRACKSC

#ifdef TRACKSC
#define BEGINF(X) std::cout << "SC: ENTER: " << X << "\n";
#define ENDF(X) std::cout << "SC: EXIT: " << X << "\n";
#define TRACKP(X) std::cout << "SC: " << X << "\n";
#else
#define BEGINF(X)
#define ENDF(X)
#define TRACKP(X)
#endif

namespace real_time
{

SearchCtrl::SearchCtrl(GlobalState const &s, LookaheadSearchMethod lsm, LearningMethod lm, DecisionStrategy ds)
	: cs(&s),
	  lb(nullptr),
	  lsm(lsm),
	  ls(nullptr),
	  lm(lm),
	  h_before_learn(lsm == LookaheadSearchMethod::BREADTH_FIRST),
	  dl(nullptr),
	  nl(nullptr),
	  ds(ds),
	  sd(nullptr),
	  dd(nullptr),
	  learning_done(true)
{}
SearchCtrl::~SearchCtrl() {}

void SearchCtrl::initialize(GlobalState const &s)
{
	cs = &s;
	BEGINF(__func__);
	ls->initialize(s);
	assert(lb != nullptr);
	// TODO: implement a way here to make a choice how long we'll
	// do look-ahead. based on how much time one iteration takes
	// at most.
	lb->initialize(*ls);
	ENDF(__func__)
}


SearchStatus SearchCtrl::search()
{
	BEGINF(__func__);
	// TODO: check flag if we need to finish learning from
	// previous iteration first.
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


void SearchCtrl::learn()
{
	// TODO: make learning steppable, interupt if out of time and set the
	// flag to continue learning next iteration
	switch (lm) {
	case LearningMethod::NONE: break;
	case LearningMethod::DIJKSTRA: dl->apply_updates(ls->get_predecessors(), ls->get_frontier(), ls->get_closed(), h_before_learn); break;
	case LearningMethod::NANCY:    nl->apply_updates(ls->get_predecessors(), ls->get_frontier(), ls->get_closed(), *cs); break;
	}
}

OperatorID SearchCtrl::select_action()
{
	switch (ds) {
	case DecisionStrategy::NANCY: return dd->pick_top_level_action(ls->get_search_space()); break;
	default:                      return sd->get_top_level_action(ls->get_frontier(), ls->get_search_space());   break;
	}
}

MaxExpansions::MaxExpansions(int b): stats(nullptr), bound(b) {}

bool MaxExpansions::ok() const
{
	assert(stats != nullptr);
	return stats->get_expanded() < bound;
}

void MaxExpansions::initialize(LookaheadSearch const &ls)
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


MaxTime::MaxTime(int ms) : max_ms(std::chrono::duration<int, std::milli>(ms)) {BEGINF(__func__); ENDF(__func__);}

bool MaxTime::ok() const
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

void MaxTime::initialize(LookaheadSearch const &ls)
{
	BEGINF(__func__);
	(void)&ls;
	timer.reset();
	ENDF(__func__);
}

void SearchCtrl::prepare_statistics()
{
	std::sort(expansions.begin(), expansions.end());
}

void SearchCtrl::print_statistics() const
{
	ls->print_statistics();

	assert(std::is_sorted(expansions.begin(), expansions.end()));
	auto const &v = expansions;
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

}

#undef BEGINF
#undef ENDF
#undef TRACKP
