#ifndef REALTIME_SEARCH_CTRL_H
#define REALTIME_SEARCH_CTRL_H

#include <memory>

#include "../search_engine.h" // for SearchStatus

#include "kinds.h"
#include "bound.h"
#include "lookhead_search.h"
#include "learning.h"
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
	BackupMethod lm;
	std::unique_ptr<Learning> le;
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
	SearchCtrl(GlobalState const &s, LookaheadSearchMethod lsm, BackupMethod lm, DecisionStrategy ds);
	virtual ~SearchCtrl();

	void initialize_lookahead(GlobalState const &s);
	void initialize_learning();
	SearchStatus search();
	void learn();
	OperatorID select_action();

	void prepare_statistics();
	void print_statistics() const;
};

}

#endif
