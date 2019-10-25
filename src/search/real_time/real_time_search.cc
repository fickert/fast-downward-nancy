#include "real_time_search.h"

#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../plugin.h"
#include "../task_utils/task_properties.h"
#include "../utils/system.h"
#include "util.h"
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

namespace real_time {

RealTimeSearch::RealTimeSearch(const options::Options &opts)
	: SearchEngine(opts),
	  current_state(state_registry.get_initial_state()),
	  num_rts_phases(0),
	  solution_cost(0),
	  gaussian_fallback_count(0),
	  sc(current_state,
	     LookaheadSearchMethod(opts.get_enum("lookahead_search")),
	     LearningMethod(opts.get_enum("learning")),
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
	if (sc.lm == LearningMethod::DIJKSTRA)
		heuristic = std::make_shared<LearningEvaluator>(heuristic);
	initialize_optional_features(opts);

	bool const store_exploration_data = sc.lm != LearningMethod::NONE;
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
	case LearningMethod::NONE:
		sc.dl = nullptr;
		sc.nl = nullptr;
		break;
	case LearningMethod::DIJKSTRA:
		sc.dl = std::make_unique<DijkstraLearning>(
			std::static_pointer_cast<LearningEvaluator>(heuristic),
			std::static_pointer_cast<LearningEvaluator>(distance_heuristic),
			state_registry,
			this);
		sc.nl = nullptr;
		break;
	case LearningMethod::NANCY:
		sc.dl = nullptr;
		auto beliefs = sc.ls->get_beliefs();
		auto post_beliefs = sc.ls->get_post_beliefs();
		sc.nl = std::make_unique<NancyLearning>(state_registry, this, beliefs, post_beliefs);
		break;
	}

	std::cout << "initializing decision strat\n";
	switch (sc.ds) {
	case DecisionStrategy::MINIMIN: {
		auto f_evaluator = std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{heuristic, std::make_shared<g_evaluator::GEvaluator>()});
		sc.sd = std::make_unique<ScalarDecider>(state_registry,
			[this, f_evaluator](const StateID &state_id, SearchSpace &lookahead_search_space) {
				const auto state = state_registry.lookup_state(state_id);
				const auto node = lookahead_search_space.get_node(state);
				auto eval_context = EvaluationContext(state, node.get_g(), false, nullptr, false);
				return eval_context.get_evaluator_value_or_infinity(f_evaluator.get());
			});
		sc.dd = nullptr;
		break;
	}
	case DecisionStrategy::BELLMAN: {
		auto f_hat_evaluator = create_f_hat_evaluator(heuristic, distance_heuristic, *heuristic_error);
		sc.sd = std::make_unique<ScalarDecider>(state_registry,
			[this, f_hat_evaluator](const StateID &state_id, SearchSpace &lookahead_search_space) {
				const auto state = state_registry.lookup_state(state_id);
				const auto node = lookahead_search_space.get_node(state);
				auto eval_context = EvaluationContext(state, node.get_g(), false, nullptr, false);
				return eval_context.get_evaluator_value_or_infinity(f_hat_evaluator.get());
			});
		sc.dd = nullptr;
		break;
	}
	case DecisionStrategy::NANCY: {
		auto const tlas = this->sc.ls->get_tlas();
		assert(tlas);
		sc.sd = nullptr;
		sc.dd = std::make_unique<DistributionDecider>(tlas, *this, state_registry, current_state.get_id());
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
			if (sc.lm == LearningMethod::DIJKSTRA)
				distance_heuristic = std::make_shared<LearningEvaluator>(distance_heuristic);
		}
		heuristic_error = std::make_unique<HeuristicError>(state_registry, heuristic, distance_heuristic);
	}
	if (sc.lsm == LookaheadSearchMethod::RISK)
		expansion_delay = std::make_unique<ExpansionDelay>(opts.get<int>("expansion_delay_window_size"));
}

void RealTimeSearch::initialize() {
	BEGINF(__func__);
	assert(heuristic);
	std::cout << "Conducting real-time search" << std::endl;

	auto eval_context = EvaluationContext(current_state, &statistics, false);
	const auto dead_end = eval_context.is_evaluator_value_infinite(heuristic.get());
	statistics.inc_evaluated_states();
	print_initial_evaluator_values(eval_context);

	if (dead_end) {
		std::cout << "Initial state is a dead end, no solution" << std::endl;
		if (heuristic->dead_ends_are_reliable())
			utils::exit_with(utils::ExitCode::SEARCH_UNSOLVABLE);
		else
			utils::exit_with(utils::ExitCode::SEARCH_UNSOLVED_INCOMPLETE);
	}

	auto node = search_space.get_node(current_state);
	node.open_initial();
	ENDF(__func__);
}

SearchStatus RealTimeSearch::step() {
	++num_rts_phases;
	sc.initialize(current_state);
	TRACKP("doing lookahead");
	const auto status = sc.search();
	TRACKP("finished lookahead");

	const SearchStatistics &current_stats = sc.ls->get_statistics();
	statistics.inc_expanded(current_stats.get_expanded());
	statistics.inc_evaluated_states(current_stats.get_evaluated_states());
	statistics.inc_evaluations(current_stats.get_evaluations());
	statistics.inc_generated(current_stats.get_generated());
	statistics.inc_generated_ops(current_stats.get_generated_ops());
	statistics.inc_reopened(current_stats.get_reopened());

	if (status == FAILED)
		return FAILED;

	if (status == SOLVED) {
		auto overall_plan = Plan();
		// NOTE: the returned plan does not correspond to the agent's actual movements as it removes cycles
		search_space.trace_path(current_state, overall_plan);
		auto plan = sc.ls->get_plan();
		for (auto op_id : plan)
			solution_cost += get_adjusted_cost(task_proxy.get_operators()[op_id]);
		overall_plan.insert(std::end(overall_plan), std::begin(plan), std::end(plan));
		set_plan(overall_plan);
		return SOLVED;
	}

	if (sc.ls->get_frontier().empty())
		return FAILED;

	TRACKP("learning/backup");
	// TODO: check if it makes a difference to select the action
	// before learning (I don't think it does).  Then we could
	// learn later, and be safer with respect to the time
	// deadline.
	sc.learn();

	TRACKP("action selection");
	OperatorID best_tla = sc.select_action();

	const auto parent_node = search_space.get_node(current_state);
	const auto op = task_proxy.get_operators()[best_tla];
	solution_cost += get_adjusted_cost(op);
	// std::cout << "executing action " << op.get_name() << " with cost " << get_adjusted_cost(op) << "\n";
	current_state = state_registry.get_successor_state(current_state, op);
	auto next_node = search_space.get_node(current_state);
	if (next_node.is_new())
		next_node.open(parent_node, op, get_adjusted_cost(op));

	return IN_PROGRESS;
}

void RealTimeSearch::print_statistics() const {
	statistics.print_detailed_statistics();
	std::cout << "Number of lookahead phases: " << num_rts_phases << std::endl;
	std::cout << "Total solution cost: " << solution_cost << std::endl;
	if (heuristic_error) {
		std::cout << "Average heuristic error: " << heuristic_error->get_average_heuristic_error() << std::endl;
		if (distance_heuristic)
			std::cout << "Average distance error: " << heuristic_error->get_average_distance_error() << std::endl;
	}
	std::cout << "Fallback to gaussian (decision strategy): " << gaussian_fallback_count << std::endl;
	sc.print_statistics();
}

static auto _parse(options::OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.document_synopsis("Lazy enforced hill-climbing", "");
	parser.add_option<std::shared_ptr<Evaluator>>("h", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance_heuristic", "distance heuristic", options::OptionParser::NONE);
	parser.add_option<int>("lookahead_bound","Lookahead bound in number of expansions.", "100");
	// TODO: check if the default value here makes any sense.
	parser.add_option<int>("time_bound","Lookahead bound in milli seconds.", "200");
	parser.add_enum_option("rtbound_type", {"EXPANSIONS", "TIME"}, "Type of bound the algorithm is running under", "EXPANSIONS");
	parser.add_enum_option("lookahead_search", {"A_STAR", "A_STAR_COLLECT", "F_HAT", "BREADTH_FIRST", "RISK"}, "Lookahead search algorithm", "A_STAR");
	parser.add_enum_option("learning", {"NONE","DIJKSTRA","NANCY"}, "What kind of learning update to perform (DIJKSTRA for heuristic values, NANCY for beliefs)", "NONE");
	parser.add_enum_option("decision_strategy", {"MINIMIN", "BELLMAN", "NANCY", "CSERNA", "K_BEST"}, "Top-level action selection strategy", "MINIMIN");
	parser.add_enum_option("feature_kind", {"JUST_H", "WITH_PARENT_H"}, "Kind of features to look up the beliefs in the data (the data format has to match)", "JUST_H");
	parser.add_enum_option("post_feature_kind", {"JUST_H", "WITH_PARENT_H"}, "Kind of features to look up the post beliefs in the data (the data format has to match)", "JUST_H");
	// parser.add_option<int>("k", "Value for k-best decision strategy", "3");
	parser.add_option<int>("expansion_delay_window_size", "Sliding average window size used for the computation of expansion delays (set this to 0 to use the global average)", "0", options::Bounds("0", ""));
	parser.add_option<std::string>("hstar_data", "file containing h* data", options::OptionParser::NONE);
	parser.add_option<std::string>("post_expansion_belief_data", "file containing post-expansion belief data", options::OptionParser::NONE);

	SearchEngine::add_options_to_parser(parser);
	const auto opts = parser.parse();

	if (parser.dry_run())
		return nullptr;
	return std::make_shared<RealTimeSearch>(opts);
}

static Plugin<SearchEngine> _plugin("real_time", _parse);
}


#undef BEGINF
#undef ENDF
