#ifndef REAL_TIME_REAL_TIME_SEARCH_H
#define REAL_TIME_REAL_TIME_SEARCH_H

#include "decision_strategy.h"
#include "dijkstra_learning.h"
#include "expansion_delay.h"
#include "heuristic_error.h"
#include "lookhead_search.h"
#include "../open_list.h"
#include "../search_engine.h"

#include "util.h"

#include <memory>

namespace real_time {

class RealTimeSearch : public SearchEngine {
	std::shared_ptr<Evaluator> heuristic;
	GlobalState current_state;

	// Statistics
	int num_rts_phases;
	int solution_cost;
	int gaussian_fallback_count;

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

	std::unique_ptr<hstar_data_type<int>> hstar_data;
	std::unique_ptr<hstar_data_type<long long>> post_expansion_belief_data;

	const bool evaluate_heuristic_when_learning;

	std::unique_ptr<LookaheadSearch> lookahead_search;
	std::unique_ptr<DijkstraLearning> dijkstra_learning;
	std::unique_ptr<real_time::DecisionStrategy> decision_strategy;

	std::shared_ptr<Evaluator> distance_heuristic;
	std::unique_ptr<ExpansionDelay> expansion_delay;
	std::unique_ptr<HeuristicError> heuristic_error;
	void initialize_optional_features(const options::Options &opts);

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
