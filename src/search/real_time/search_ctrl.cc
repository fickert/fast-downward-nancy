#include "search_ctrl.h"

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

SearchCtrl::SearchCtrl() : ls(nullptr), lb(nullptr) {}
SearchCtrl::~SearchCtrl() {}

void SearchCtrl::initialize(GlobalState const &s)
{
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
	switch (learning_method) {
	case LearningMethod::NONE:
		break;
	case LearningMethod::DIJKSTRA:
		dijkstra_learning->apply_updates(lc.ls->get_predecessors(), lc.ls->get_frontier(), lc.ls->get_closed(), evaluate_heuristic_when_learning);
		break;
	case LearningMethod::NANCY:
		nancy_learning->apply_updates(lc.ls->get_predecessors(), lc.ls->get_frontier(), lc.ls->get_closed(), current_state);
		break;
	}

}

void SearchCtrl::act()
{
	OperatorID best_tla(0);
	switch (decision_strategy_type) {
	case DecisionStrategy::NANCY:
		assert(nancy_decision_strategy);
		best_tla = dd->pick_top_level_action(lc.ls->get_search_space());
		break;
	default:
		best_tla = sd->get_top_level_action(lc.ls->get_frontier(), lc.ls->get_search_space());
		break;
	}

	const auto parent_node = search_space.get_node(current_state);
	const auto op = task_proxy.get_operators()[best_tla];
	solution_cost += get_adjusted_cost(op);
	// std::cout << "executing action " << op.get_name() << " with cost " << get_adjusted_cost(op) << "\n";
	current_state = state_registry.get_successor_state(current_state, op);
	auto next_node = search_space.get_node(current_state);
	if (next_node.is_new())
		next_node.open(parent_node, op, get_adjusted_cost(op));
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




void SearchCtrl::print_statistics() const
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

]

#undef BEGINF
#undef ENDF
#undef TRACKP
