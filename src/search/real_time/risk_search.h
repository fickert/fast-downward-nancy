#ifndef RISK_SEARCH_H
#define RISK_SEARCH_H

#include "lookhead_search.h"
#include "DiscreteDistribution.h"
#include "belief_data.h"
#include "belief_store.h"
#include "tlas.h"
#include "../evaluator.h"
#include "../open_list.h"
#include "../search_engine.h"
#include "../search_space.h"
#include "../state_registry.h"
#include "../per_state_information.h"

#include <memory>
#include <vector>
#include <unordered_map>

namespace real_time
{

class RiskLookaheadSearch : public LookaheadSearch
{
	using Beliefs = PerStateInformation<ShiftedDistribution>;

	std::shared_ptr<Evaluator> f_evaluator;
	std::shared_ptr<Evaluator> f_hat_evaluator;
	std::shared_ptr<Evaluator> heuristic;
	std::shared_ptr<Evaluator> base_heuristic;
	std::shared_ptr<Evaluator> distance_heuristic;

	TLAs tlas;

	// beliefs for each state
	Beliefs beliefs;
	Beliefs post_beliefs;
	// cached distributions for each feature value
	BeliefStore<int> raw_beliefs;
	BeliefStore<long long> raw_post_beliefs;
	// the kind of feature values used to lookup beliefs
	DataFeatureKind f_kind;
	DataFeatureKind pf_kind;
	ShiftedDistribution dead_end_belief;
	// raw data to make h* distributions
	HStarData<int> *hstar_data;
	HStarData<long long> *post_expansion_belief_data;

	int hstar_gaussian_fallback_count;
	int post_expansion_belief_gaussian_fallback_count;
	int alpha_expansion_count;
	int beta_expansion_count;

	// stores the index of the tla that owns the state
	// this is a hack to detect and prevent a state being expanded
	// under a tla when there is a different tla that has a shorter
	// path to that state.
	std::unordered_map<StateID, std::vector<int> > state_owners;

	// This is storage for applicable operators
	// It's kept here in the class because clearing a vector is
	// more efficient than creating a new one each iteration
	std::vector<OperatorID> applicables;
protected:
	//std::unique_ptr<StateOpenList> create_open_list() const;
	bool add_state_owner(StateID state_id, int tla_id);
	void make_state_owner(StateID state_id, int tla_id);
	bool state_owned_by_tla(StateID state_id, int tla_id) const;
	double risk_analysis(std::size_t const alpha, const vector<ShiftedDistribution> &beliefs) const;
	std::size_t select_tla();
	void backup_beliefs();
	ShiftedDistribution get_belief(EvaluationContext &context, int ph);
	ShiftedDistribution get_post_belief(StateID state_id);
public:

	RiskLookaheadSearch(StateRegistry &state_registry,
		std::shared_ptr<Evaluator> heuristic,
		std::shared_ptr<Evaluator> base_heuristic,
		std::shared_ptr<Evaluator> distance,
		bool store_exploration_data,
		ExpansionDelay *expansion_delay,
		HeuristicError *heuristic_error,
		HStarData<int> *hstar_data,
		HStarData<long long> *post_expansion_belief_data,
		SearchEngine const *search_engine,
		DataFeatureKind f_kind,
		DataFeatureKind pf_kind);
	~RiskLookaheadSearch() override;

	void initialize(const GlobalState &initial_state) final;
	auto step() -> SearchStatus final;
	auto post() -> void final;

	void print_statistics() const override;

	RiskLookaheadSearch(const RiskLookaheadSearch &) = delete;
	RiskLookaheadSearch(RiskLookaheadSearch &&) = delete;

	auto operator=(const RiskLookaheadSearch &) = delete;
	auto operator=(RiskLookaheadSearch &&) = delete;

	auto get_tlas() -> TLAs const * { return &tlas; }
	auto get_beliefs() -> PerStateInformation<ShiftedDistribution> * { return &beliefs; }
	auto get_post_beliefs() -> PerStateInformation<ShiftedDistribution> * { return &post_beliefs; }
};

}


#endif
