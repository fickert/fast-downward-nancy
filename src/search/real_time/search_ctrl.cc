#include "search_ctrl.h"

#include <numeric>   // accumulate
#include <algorithm> // sort
#include <chrono>    // the search control itself also measures time
#include <tuple>

#include "vec_stats.h"
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
{
	durations.reserve(1 << 18); // 262144, should be plenty for most instances
}
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
		auto dur = ls->stop_lap();
		//durations.push_back((std::chrono::duration_cast<std::chrono::nanoseconds>(dur)).count());
	}
	expansions.push_back(ls->get_statistics().get_expanded());
	TRACKP("expanded " << ls->get_statistics().get_expanded());
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
	// std::sort(durations.begin(), durations.end());
}

// template<typename T>
// std::tuple<T,T,T,T> vec_stats(std::vector<T> const &v)
// {
// }

void SearchCtrl::print_statistics() const
{
	ls->print_statistics();

	auto estats = vec_stats(expansions);

	std::cout << "Average number of expansions: " << estats.avg << "\n"
		  << "Minimum number of expansions: " << estats.min << "\n"
		  << "Maximum number of expansions: " << estats.max << "\n"
		  << "Median expansions: " << estats.med << "\n"
		  << "Number of catchup learning phases: " << catchups << "\n";
	//auto dstats = vec_stats(durations);
	// std::cout << "Average lookahead iteration duration: " << dstats.avg << "ns\n"
	// 	  << "Minimum lookahead iteration duration: " << dstats.min << "ns\n"
	// 	  << "Maximum lookahead iteration duration: " << dstats.max << "ns\n"
	// 	  << "Median lookahead iteration duration: " << dstats.med << "ns\n";




}

}

#undef BEGINF
#undef ENDF
#undef TRACKP
