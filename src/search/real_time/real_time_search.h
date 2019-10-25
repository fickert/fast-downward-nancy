#ifndef REAL_TIME_REAL_TIME_SEARCH_H
#define REAL_TIME_REAL_TIME_SEARCH_H

#include "kinds.h"
#include "decision_strategy.h"
#include "dijkstra_learning.h"
#include "nancy_learning.h"
#include "expansion_delay.h"
#include "heuristic_error.h"
#include "lookhead_search.h"
#include "search_ctrl.h"
#include "../open_list.h"
#include "../search_engine.h"

#include "util.h"

#include <memory>

namespace real_time {

class RealTimeSearch : public SearchEngine {
	std::shared_ptr<Evaluator> heuristic;
	std::shared_ptr<Evaluator> base_heuristic;
	GlobalState current_state;

	// Statistics
	int num_rts_phases;
	int solution_cost;
	int gaussian_fallback_count;

	std::unique_ptr<HStarData<int>> hstar_data;
	std::unique_ptr<HStarData<long long>> post_expansion_belief_data;

	SearchCtrl sc;

	std::shared_ptr<Evaluator> distance_heuristic;
	std::unique_ptr<ExpansionDelay> expansion_delay;
	std::unique_ptr<HeuristicError> heuristic_error;
	void initialize_optional_features(const options::Options &opts);

protected:
	void initialize() override;
	auto step() -> SearchStatus override;

public:
	explicit RealTimeSearch(const options::Options &opts);
	~RealTimeSearch() override {};
	auto get_expanded_states() -> std::unique_ptr<std::unordered_set<StateID> > override { return std::move(sc.ls->get_expanded_states()); }

	void print_statistics() const override;
};
}

#endif
