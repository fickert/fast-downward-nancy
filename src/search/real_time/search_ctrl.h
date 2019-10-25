#ifndef REALTIME_SEARCH_CTRL_H
#define REALTIME_SEARCH_CTRL_H

#include <memory>
#include <chrono> // for a more accurate timer
#include "lookhead_search.h"
#include "../utils/timer.h"
#include "../search_engine.h" // for SearchStatus

namespace real_time
{

// A lookahead bound is a certain type of bound imposed
struct Bound;
// Lookahead Control
struct SearchControl;

struct SearchControl
{
	std::unique_ptr<Bound> lb;
	std::unique_ptr<LookaheadSearch> ls;
	LearningMethod learning_method;
	std::unique_ptr<DijkstraLearning> dl;
	std::unique_ptr<NancyLearning> nl;
	DecisionStrategy decision_strategy_type;
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
	LookaheadControl();
	virtual ~LookaheadControl();

	void initialize(GlobalState const &s);
	SearchStatus search();
	void learn();
	void act();

	void print_statistics() const;
};

struct Bound
{
	virtual ~LookaheadBound() = default;
	virtual bool ok() const = 0;
	virtual void initialize(LookaheadSearch const &ls) = 0;
};

struct MaxExpansions : Bound
{
	SearchStatistics const *stats;
	int bound;

	ExpansionBound(int b);
	virtual ~ExpansionBound() = default;

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

	TimeBound(int max_ms);
	virtual ~TimeBound() = default;

	bool ok() const final;
	void initialize(LookaheadSearch const &ls) final;
};

}

#endif
