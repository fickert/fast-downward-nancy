#ifndef ONLINE_RISK_SEARCH_H
#define ONLINE_RISK_SEARCH_H

#include "lookhead_search.h"
#include "DiscreteDistribution.h"
#include "../evaluator.h"
#include "../open_list.h"
#include "../search_engine.h"
#include "../search_space.h"
#include "../state_registry.h"

#include <memory>
#include <vector>
#include <unordered_map>

namespace real_time
{

struct OnlineTLAs
{
	using Cost = double;
	std::vector<OperatorID> ops;
	std::vector<std::unique_ptr<StateOpenList>> open_lists;
	std::vector<DiscreteDistribution> beliefs;
	std::vector<Cost> expected_min_costs;
	std::vector<StateID> states;
	/*
	// "states" stores the top most states of each tla.
	// if a1,a2,a3 are the tlas, then "states" stores the state ids of the nodes n1,n2,n3:
	current_node
	/ | \
	a1   a2  a3 ...
        /     |     \
	n1    n2     n3 ...
	*/

	// TODO: not sure if this is smart, setting up all these eval
	// contexts
	std::vector<EvaluationContext> eval_contexts;


	void reserve(std::size_t n);
	void clear();
	size_t size() const;
};

struct DParms
{
	int mean;
	int d;
	int error;
	DParms(int a, int b, int c) : mean(a), d(b), error(c) {}
	~DParms() {}
};

class OnlineRiskLookaheadSearch : public LookaheadSearch
{
	std::shared_ptr<Evaluator> f_evaluator;
	std::shared_ptr<Evaluator> f_hat_evaluator;
	std::shared_ptr<Evaluator> heuristic;
	std::shared_ptr<Evaluator> base_heuristic;
	std::shared_ptr<Evaluator> distance_heuristic;
	OnlineTLAs tlas;

	int hstar_gaussian_fallback_count;
	int post_expansion_belief_gaussian_fallback_count;

	// stores the index of the tla that owns the state
	// this is a hack to detect and prevent a state being expanded
	// under a tla when there is a different tla that has a shorter
	// path to that state.
	std::unordered_map<StateID, int> state_owner;

	// This is storage for applicable operators
	// It's kept here in the class because clearing a vector is
	// more efficient than creating a new one each iteration
	std::vector<OperatorID> applicables;

	// This is just for statistics on the generated distribution
	bool collect_dparms;
	std::unordered_map<int, std::vector<DParms>> dparms;
protected:
	bool state_owned_by_tla(StateID state_id, int tla_id) const;
	void generate_tlas(GlobalState const &current_state);
	std::unique_ptr<StateOpenList> create_open_list() const;
	double risk_analysis(std::size_t const alpha, const vector<DiscreteDistribution> &squished_beliefs) const;
	std::size_t select_tla();
	void backup_beliefs();
	DiscreteDistribution node_belief(SearchNode const &);
	void post_expansion_belief(StateID best_state_id, DiscreteDistribution &current_belief);
public:

	OnlineRiskLookaheadSearch(StateRegistry &state_registry,
		std::shared_ptr<Evaluator> heuristic,
		std::shared_ptr<Evaluator> base_heuristic,
		std::shared_ptr<Evaluator> distance,
		bool store_exploration_data,
		ExpansionDelay *expansion_delay,
		HeuristicError *heuristic_error,
		SearchEngine const *se,
		bool collect_dparms);
	~OnlineRiskLookaheadSearch() override = default;

	void initialize(const GlobalState &initial_state) override;
	//auto search() -> SearchStatus override;
	auto step() -> SearchStatus override;
	auto post() -> void override;

	void print_statistics() const override;

	OnlineRiskLookaheadSearch(const OnlineRiskLookaheadSearch &) = delete;
	OnlineRiskLookaheadSearch(OnlineRiskLookaheadSearch &&) = delete;

	auto operator=(const OnlineRiskLookaheadSearch &) = delete;
	auto operator=(OnlineRiskLookaheadSearch &&) = delete;
};

}


#endif
