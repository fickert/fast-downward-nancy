#include "search_ctrl.h"

#include <numeric>   // accumulate
#include <algorithm> // sort
#include <chrono>    // the search control itself also measures time

#include "lap_timer.h"

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
	  dec(nullptr),
	  learning_done(true)
{}
SearchCtrl::~SearchCtrl() {}

void SearchCtrl::initialize_lookahead(GlobalState const &s)
{
	if (!learning_done) {
		TRACKP("finishing learning from before");
		learn_catch_up();
	}

	cs = &s;
	BEGINF(__func__);
	ls->initialize(s);
	assert(lb != nullptr);
	lb->initialize(*ls);
	ENDF(__func__)
}

SearchStatus SearchCtrl::search()
{
	BEGINF(__func__);

	SearchStatus res = IN_PROGRESS;
	while (lb->lookahead_ok() && res == IN_PROGRESS) {
		ls->next_lap();
		res = ls->step();
		ls->stop_lap();
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


void SearchCtrl::learn_catch_up()
{
	while (lb->learning_ok() && !le->done()) {
		le->step();
	}
}

void SearchCtrl::learn_initial()
{
	BEGINF(__func__);
	// build up the learning queue
	TRACKP("initializing learning queue");
	le->initialize(ls->get_predecessors(), ls->get_frontier(), ls->get_closed());

	// do it
	TRACKP("doing learning");
	while (lb->learning_ok() && !le->done()) {
		le->step();
	}
	learning_done = le->done();
	ENDF(__func__);
}

OperatorID SearchCtrl::select_action()
{
	return dec->decide(ls->get_frontier(), ls->get_search_space());
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
		  << "Median expansions: " << med << "\n"
		  << "Number of catchup learning phases: " << catchups << "\n";
}

}

#undef BEGINF
#undef ENDF
#undef TRACKP
