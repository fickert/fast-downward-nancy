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

SearchCtrl::SearchCtrl(GlobalState const &s, LookaheadSearchMethod lsm, BackupMethod lm, DecisionStrategy ds)
	: cs(&s),
	  lb(nullptr),
	  lsm(lsm),
	  ls(nullptr),
	  lm(lm),
	  le(nullptr),
	  ds(ds),
	  sd(nullptr),
	  dd(nullptr),
	  learning_done(true)
{}
SearchCtrl::~SearchCtrl() {}

void SearchCtrl::initialize_lookahead(GlobalState const &s)
{
	cs = &s;
	BEGINF(__func__);
	ls->initialize(s);
	assert(lb != nullptr);
	lb->initialize(*ls);
	ENDF(__func__)
}

void SearchCtrl::initialize_learning()
{
	BEGINF(__func__);
	le->initialize(ls->get_predecessors(), ls->get_frontier(), ls->get_closed());
	ENDF(__func__)
}

SearchStatus SearchCtrl::search()
{
	BEGINF(__func__);
	if (!learning_done) {
		// TODO: implement
	}

	SearchStatus res = IN_PROGRESS;
	while (lb->lookahead_ok() && res == IN_PROGRESS) {
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
	auto effort = le->effort();
	while (lb->learning_ok() && !le->done()) {
		le->step();
	}
	learning_done = le->done();
	auto remaining = le->remaining();

	auto done = effort - remaining;
	if (0 == done) {
		TRACKP("didn't finish a single entry during learning");
	} else {
		TRACKP("in learning remained " << remaining << " out of " << effort);
	}

	lb->adjust_learning(effort, remaining);
}

OperatorID SearchCtrl::select_action()
{
	switch (ds) {
	case DecisionStrategy::NANCY: return dd->pick_top_level_action(ls->get_search_space()); break;
	default:                      return sd->get_top_level_action(ls->get_frontier(), ls->get_search_space());   break;
	}
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
