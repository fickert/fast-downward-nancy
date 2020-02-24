#include "search_ctrl.h"

#include <numeric>   // accumulate
#include <algorithm> // sort
#include <chrono>    // the search control itself also measures time
#include <tuple>

#include "vec_stats.h"
#include "lap_timer.h"

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
{
	durations.reserve(1 << 18); // 262144, should be plenty for most instances
}
SearchCtrl::~SearchCtrl() {}

void SearchCtrl::initialize_lookahead(GlobalState const &s)
{
	if (!learning_done) {
		learn_catch_up();
	}

	cs = &s;
	ls->initialize(s);
	assert(lb != nullptr);
	lb->initialize(*ls);
}

SearchStatus SearchCtrl::search()
{
	SearchStatus res = IN_PROGRESS;
	while (lb->lookahead_ok() && res == IN_PROGRESS) {
		ls->next_lap();
		res = ls->step();
		auto dur = ls->stop_lap();
	}
	expansions.push_back(ls->get_statistics().get_expanded());
	switch (res) {
	case FAILED:        return FAILED;
	case SOLVED:        return SOLVED;
	case IN_PROGRESS:   ls->post(); return IN_PROGRESS;
	case TIMEOUT:       return TIMEOUT;
	}

#ifdef __GNUC__
	__builtin_unreachable();
#endif
	utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
}


void SearchCtrl::learn_catch_up()
{
	while (lb->learning_ok() && !le->done()) {
		le->step();
	}
}

void SearchCtrl::learn_initial()
{
	// build up the learning queue
	le->initialize(ls->get_predecessors(), ls->get_frontier(), ls->get_closed());

	while (lb->learning_ok() && !le->done()) {
		le->step();
	}
	learning_done = le->done();
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

	auto estats = vec_stats(expansions);

	std::cout << "Average number of expansions: " << estats.avg << "\n"
		  << "Minimum number of expansions: " << estats.min << "\n"
		  << "Maximum number of expansions: " << estats.max << "\n"
		  << "Median expansions: " << estats.med << "\n"
		  << "Number of catchup learning phases: " << catchups << "\n";
}

}
