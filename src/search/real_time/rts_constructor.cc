#include "real_time_search.h"

#include "../task_utils/task_properties.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../options/options.h"
#include "learning_evaluator.h"
#include "max_time.h"
#include "max_expansions.h"
#include "nancy_backup.h"
#include "dijkstra_backup.h"
#include "scalar_decider.h"
#include "dist_decider.h"
#include "belief_data.h"
#include "DiscreteDistribution.h"
#include "state_collector.h"
#include "risk_search.h"

#include <iostream>

// #define TRACKRT

#ifdef TRACKRT
#define BEGINF(X) std::cout << "RT: ENTER: " << X << "\n";
#define ENDF(X) std::cout << "RT: EXIT: " << X << "\n";
#define TRACKP(X) std::cout << "RT: " << X << "\n";
#else
#define BEGINF(X)
#define ENDF(X)
#define TRACKP(X)
#endif


namespace real_time
{

RealTimeSearch::RealTimeSearch(const options::Options &opts)
	: SearchEngine(opts),
	  current_state(state_registry.get_initial_state()),
	  num_rts_phases(0),
	  solution_cost(0),
	  gaussian_fallback_count(0),
	  sc(current_state,
	     LookaheadSearchMethod(opts.get_enum("lookahead_search")),
	     BackupMethod(opts.get_enum("learning")),
	     DecisionStrategy(opts.get_enum("decision_strategy")))
{
	BEGINF(__func__);
	DataFeatureKind f_kind = static_cast<DataFeatureKind>(opts.get_enum("feature_kind"));
	DataFeatureKind pf_kind = static_cast<DataFeatureKind>(opts.get_enum("post_feature_kind"));
	if (opts.contains("hstar_data"))
		hstar_data = std::make_unique<HStarData<int>>(opts.get<std::string>("hstar_data"), f_kind);
	if (opts.contains("post_expansion_belief_data"))
		post_expansion_belief_data = std::make_unique<HStarData<long long>>(opts.get<std::string>("post_expansion_belief_data"), pf_kind);

	heuristic = opts.get<std::shared_ptr<Evaluator>>("h");
	base_heuristic = heuristic;
	// just so I don't have to use dynamic_cast later
	if (sc.lm == BackupMethod::DIJKSTRA) {
		heuristic = std::make_shared<LearningEvaluator>(heuristic);
	}

	initialize_optional_features(opts);

	bool const store_exploration_data = sc.lm != BackupMethod::NONE;
	//bool const store_exploration_data = true;

	std::cout << "initializing lookahead method\n";
	switch (sc.lsm) {
	case LookaheadSearchMethod::A_STAR_COLLECT:
		sc.ls = std::make_unique<AStarCollect>(state_registry, opts.get<int>("lookahead_bound"), heuristic, store_exploration_data, expansion_delay.get(), heuristic_error.get(), this);
		break;
	case LookaheadSearchMethod::A_STAR:
		sc.ls = std::make_unique<AStarLookaheadSearch>(state_registry, opts.get<int>("lookahead_bound"), heuristic, store_exploration_data, expansion_delay.get(), heuristic_error.get(), this);
		break;
	case LookaheadSearchMethod::BREADTH_FIRST:
		sc.ls = std::make_unique<BreadthFirstLookaheadSearch>(state_registry, opts.get<int>("lookahead_bound"), store_exploration_data, expansion_delay.get(), heuristic_error.get(), this);
		break;
	case LookaheadSearchMethod::F_HAT:
		sc.ls = std::make_unique<FHatLookaheadSearch>(state_registry, opts.get<int>("lookahead_bound"), heuristic, distance_heuristic, store_exploration_data, expansion_delay.get(), *heuristic_error, this);
		break;
	case LookaheadSearchMethod::RISK:
		sc.ls = std::make_unique<RiskLookaheadSearch>(state_registry, opts.get<int>("lookahead_bound"), heuristic, base_heuristic, distance_heuristic, store_exploration_data, expansion_delay.get(), heuristic_error.get(), hstar_data.get(), post_expansion_belief_data.get(), this, f_kind, pf_kind);
		break;
	default:
		std::cerr << "unknown lookahead search method: " << opts.get_enum("lookahead_search") << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
	}

	std::cout << "initializing lookahead termination condition\n";
	switch (BoundKind(opts.get_enum("rtbound_type"))) {
	case BoundKind::EXPANSIONS:
		sc.lb = std::make_unique<MaxExpansions>(opts.get<int>("lookahead_bound"));
		break;
	case BoundKind::TIME:
		sc.lb = std::make_unique<MaxTime>(opts.get<int>("time_bound"));
		break;
	}

	std::cout << "initializing learning method\n";
	switch (sc.lm) {
	case BackupMethod::NONE:
		sc.le = nullptr;
		break;
	case BackupMethod::DIJKSTRA:
		sc.le = std::make_unique<DijkstraBackup>(state_registry, this, dynamic_pointer_cast<LearningEvaluator>(heuristic), dynamic_pointer_cast<LearningEvaluator>(distance_heuristic), sc.lsm != LookaheadSearchMethod::BREADTH_FIRST);
		break;
	case BackupMethod::NANCY:
		auto beliefs = sc.ls->get_beliefs();
		auto post_beliefs = sc.ls->get_post_beliefs();
		sc.le = std::make_unique<NancyBackup>(state_registry, this, beliefs, post_beliefs);
		break;
	}

	std::cout << "initializing decision strat\n";
	switch (sc.ds) {
	case DecisionStrategy::MINIMIN: {
		auto f_evaluator = std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{heuristic, std::make_shared<g_evaluator::GEvaluator>()});
		sc.dec = std::make_unique<ScalarDecider>(state_registry,
			[this, f_evaluator](const StateID &state_id, SearchSpace &lookahead_search_space) {
				const auto state = state_registry.lookup_state(state_id);
				const auto node = lookahead_search_space.get_node(state);
				auto eval_context = EvaluationContext(state, node.get_g(), false, nullptr, false);
				return eval_context.get_evaluator_value_or_infinity(f_evaluator.get());
			});
		break;
	}
	case DecisionStrategy::BELLMAN: {
		auto f_hat_evaluator = create_f_hat_evaluator(heuristic, distance_heuristic, *heuristic_error);
		sc.dec = std::make_unique<ScalarDecider>(state_registry,
			[this, f_hat_evaluator](const StateID &state_id, SearchSpace &lookahead_search_space) {
				const auto state = state_registry.lookup_state(state_id);
				const auto node = lookahead_search_space.get_node(state);
				auto eval_context = EvaluationContext(state, node.get_g(), false, nullptr, false);
				return eval_context.get_evaluator_value_or_infinity(f_hat_evaluator.get());
			});
		break;
	}
	case DecisionStrategy::NANCY: {
		auto const tlas = this->sc.ls->get_tlas();
		assert(tlas);
		sc.dec = std::make_unique<DistributionDecider>(state_registry, tlas, *this, current_state.get_id());
		break;
	}
	default:
		std::cerr << "unknown decision strategy: " << opts.get_enum("decision_strategy") << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
	}

	heuristic->notify_initial_state(current_state);
	ENDF(__func__);
}

void RealTimeSearch::initialize_optional_features(const options::Options &opts) {
	if (sc.lsm == LookaheadSearchMethod::F_HAT ||
	    sc.lsm == LookaheadSearchMethod::RISK ||
	    sc.ds != DecisionStrategy::MINIMIN)
	{
		if (task_properties::is_unit_cost(task_proxy) || opts.get<std::shared_ptr<Evaluator>>("h") == opts.get<std::shared_ptr<Evaluator>>("distance_heuristic")) {
			distance_heuristic = heuristic;
		} else {
			distance_heuristic = opts.get<std::shared_ptr<Evaluator>>("distance_heuristic");
			if (sc.lm == BackupMethod::DIJKSTRA)
				distance_heuristic = std::make_shared<LearningEvaluator>(distance_heuristic);
		}
		heuristic_error = std::make_unique<HeuristicError>(state_registry, heuristic, distance_heuristic);
	}
	if (sc.lsm == LookaheadSearchMethod::RISK)
		expansion_delay = std::make_unique<ExpansionDelay>(opts.get<int>("expansion_delay_window_size"));
}

RealTimeSearch::~RealTimeSearch()
{
}

}

#undef BEGINF
#undef ENDF
#undef TRACKP
