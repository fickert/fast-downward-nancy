#ifndef REAL_TIME_REAL_TIME_SEARCH_H
#define REAL_TIME_REAL_TIME_SEARCH_H

#include "decision_strategy.h"
#include "dijkstra_learning.h"
#include "lookhead_search.h"
#include "../open_list.h"
#include "../search_engine.h"

#include <memory>

namespace real_time {

class RealTimeSearch : public SearchEngine {
	std::shared_ptr<Evaluator> heuristic;
	GlobalState current_state;

	// Statistics
	int num_rts_phases;

	enum class LookaheadSearchMethod {
		A_STAR,
		F_HAT,
		BREADTH_FIRST,
		RISK
	};

	enum class DecisionStrategy {
		MINIMIN,
		BELLMAN,
		NANCY,
		CSERNA,
		K_BEST
	};

	std::unique_ptr<LookaheadSearch> lookahead_search;
	std::unique_ptr<DijkstraLearning> dijkstra_learning;
	std::unique_ptr<real_time::DecisionStrategy> decision_strategy;

protected:
	void initialize() override;
	auto step() -> SearchStatus override;

public:
	explicit RealTimeSearch(const options::Options &opts);
	~RealTimeSearch() override = default;

	void print_statistics() const override;
};
}

#endif
