#ifndef REALTIME_SEARCH_CTRL_H
#define REALTIME_SEARCH_CTRL_H

#include <memory>
#include <chrono> // for a more accurate timer
#include "../utils/timer.h"
#include "../search_engine.h" // for SearchStatus

#include "kinds.h"
#include "lookhead_search.h"
#include "dijkstra_learning.h"
#include "nancy_learning.h"
#include "decision_strategy.h"

namespace real_time
{

// A lookahead bound is a certain type of bound imposed
struct Bound;
// Lookahead Control
struct SearchCtrl;

struct SearchCtrl
{
	GlobalState const *cs;
	std::unique_ptr<Bound> lb;
	LookaheadSearchMethod lsm;
	std::unique_ptr<LookaheadSearch> ls;
	LearningMethod lm;
	const bool h_before_learn;
	std::unique_ptr<DijkstraLearning> dl;
	std::unique_ptr<NancyLearning> nl;
	DecisionStrategy ds;
	std::unique_ptr<ScalarDecider> sd;
	std::unique_ptr<DistributionDecider> dd;
	// debug statistics.  this vector collects the number of
	// expansions in each lookahead phase.  interesting to look at
	// when a time bound is used.
	std::vector<int> expansions;

	// we select an action first, then do the learning. if we
	// couldn't finish it, we gotta do it next iteration before
	// the lookahead phase.
	bool learning_done;

	// TODO: allow in place construction maybe
	SearchCtrl(GlobalState const &s, LookaheadSearchMethod lsm, LearningMethod lm, DecisionStrategy ds);
	virtual ~SearchCtrl();

	void initialize(GlobalState const &s);
	SearchStatus search();
	void learn();
	OperatorID select_action();

	void prepare_statistics();
	void print_statistics() const;
};

struct Bound
{
	virtual ~Bound() = default;
	virtual bool ok() const = 0;
	virtual void initialize(LookaheadSearch const &ls) = 0;
};

struct MaxExpansions : Bound
{
	SearchStatistics const *stats;
	int bound;

	MaxExpansions(int b);
	virtual ~MaxExpansions() = default;

	bool ok() const final;
	void initialize(LookaheadSearch const &ls) final;
};

struct RtTimer
{
	std::chrono::time_point<std::chrono::system_clock> start;

	RtTimer();
	~RtTimer();

	std::chrono::duration<int, std::milli> diff() const;
	void reset();
};

struct MaxTime : Bound
{
	const std::chrono::duration<int, std::milli> max_ms;
	RtTimer timer;

	MaxTime(int max_ms);
	virtual ~MaxTime() = default;

	bool ok() const final;
	void initialize(LookaheadSearch const &ls) final;
};

}

#endif
