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
#define BEGINF(X) std::cout << "ENTER: " << X << "\n";
#define ENDF(X) std::cout << "EXIT: " << X << "\n";
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
	  evaluate_heuristic_when_learning(LookaheadSearchMethod(opts.get_enum("lookahead_search")) == LookaheadSearchMethod::BREADTH_FIRST),
    learning_method(LearningMethod(opts.get_enum("learning")))
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
	if (learning_method == LearningMethod::DIJKSTRA)
		heuristic = std::make_shared<LearningEvaluator>(heuristic);
	initialize_optional_features(opts);

	bool const store_exploration_data = learning_method != LearningMethod::NONE;
	//bool const store_exploration_data = true;

	std::cout << "initializing lookahead termination condition\n";
	switch (Bound(opts.get_enum("lookahead_term"))) {
	case Bound::EXPANSIONS:
		lc.lb = std::make_unique<ExpansionBound>(statistics, opts.get<int>("lookahead_bound"));
		break;
	case Bound::TIME:
		lc.lb = std::make_unique<TimeBound>(opts.get<double>("time_bound"));
		break;
	}

	std::cout << "initializing lookahead method\n";
	switch (LookaheadSearchMethod(opts.get_enum("lookahead_search"))) {
	case LookaheadSearchMethod::A_STAR_COLLECT:
		lc.ls = std::make_unique<AStarCollect>(state_registry, opts.get<int>("lookahead_bound"), heuristic, store_exploration_data, expansion_delay.get(), heuristic_error.get(), this);
		break;
	case LookaheadSearchMethod::A_STAR:
		lc.ls = std::make_unique<AStarLookaheadSearch>(state_registry, opts.get<int>("lookahead_bound"), heuristic, store_exploration_data, expansion_delay.get(), heuristic_error.get(), this);
		break;
	case LookaheadSearchMethod::BREADTH_FIRST:
		lc.ls = std::make_unique<BreadthFirstLookaheadSearch>(state_registry, opts.get<int>("lookahead_bound"), store_exploration_data, expansion_delay.get(), heuristic_error.get(), this);
		break;
	case LookaheadSearchMethod::F_HAT:
		lc.ls = std::make_unique<FHatLookaheadSearch>(state_registry, opts.get<int>("lookahead_bound"), heuristic, distance_heuristic, store_exploration_data, expansion_delay.get(), *heuristic_error, this);
		break;
	case LookaheadSearchMethod::RISK:
		lc.ls = std::make_unique<RiskLookaheadSearch>(state_registry, opts.get<int>("lookahead_bound"), heuristic, base_heuristic, distance_heuristic, store_exploration_data, expansion_delay.get(), heuristic_error.get(), hstar_data.get(), post_expansion_belief_data.get(), this, f_kind, pf_kind);
		break;
	default:
		std::cerr << "unknown lookahead search method: " << opts.get_enum("lookahead_search") << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
	}

	std::cout << "initializing learning method\n";
	switch (learning_method) {
	case LearningMethod::NONE:
		dijkstra_learning = nullptr;
		break;
	case LearningMethod::DIJKSTRA:
		dijkstra_learning = std::make_unique<DijkstraLearning>(
			std::static_pointer_cast<LearningEvaluator>(heuristic),
			std::static_pointer_cast<LearningEvaluator>(distance_heuristic),
			state_registry,
			this);
		break;
	case LearningMethod::NANCY:
		auto beliefs = lc.ls->get_beliefs();
		auto post_beliefs = lc.ls->get_post_beliefs();
		nancy_learning = std::make_unique<NancyLearning>(state_registry, this, beliefs, post_beliefs);
		break;
	}

	std::cout << "initializing decision strat\n";
	decision_strategy_type = DecisionStrategy(opts.get_enum("decision_strategy"));
	switch (decision_strategy_type) {
	case DecisionStrategy::MINIMIN: {
		auto f_evaluator = std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{heuristic, std::make_shared<g_evaluator::GEvaluator>()});
		decision_strategy = std::make_unique<ScalarDecisionStrategy>(state_registry,
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
		decision_strategy = std::make_unique<ScalarDecisionStrategy>(state_registry,
			[this, f_hat_evaluator](const StateID &state_id, SearchSpace &lookahead_search_space) {
				const auto state = state_registry.lookup_state(state_id);
				const auto node = lookahead_search_space.get_node(state);
				auto eval_context = EvaluationContext(state, node.get_g(), false, nullptr, false);
				return eval_context.get_evaluator_value_or_infinity(f_hat_evaluator.get());
			});
		break;
	}
	case DecisionStrategy::NANCY: {
		auto const tlas = this->lc.ls->get_tlas();
		assert(tlas);
		nancy_decision_strategy = std::make_unique<NancyDecisionStrategy>(tlas, *this, state_registry, current_state.get_id());
		decision_strategy = nullptr;
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
	if (LookaheadSearchMethod(opts.get_enum("lookahead_search")) == LookaheadSearchMethod::F_HAT ||
		LookaheadSearchMethod(opts.get_enum("lookahead_search")) == LookaheadSearchMethod::RISK ||
		DecisionStrategy(opts.get_enum("decision_strategy")) != DecisionStrategy::MINIMIN) {
		if (task_properties::is_unit_cost(task_proxy) || opts.get<std::shared_ptr<Evaluator>>("h") == opts.get<std::shared_ptr<Evaluator>>("distance_heuristic")) {
			distance_heuristic = heuristic;
		} else {
			distance_heuristic = opts.get<std::shared_ptr<Evaluator>>("distance_heuristic");
			if (learning_method == LearningMethod::DIJKSTRA)
				distance_heuristic = std::make_shared<LearningEvaluator>(distance_heuristic);
		}
		heuristic_error = std::make_unique<HeuristicError>(state_registry, heuristic, distance_heuristic);
	}
	if (LookaheadSearchMethod(opts.get_enum("lookahead_search")) == LookaheadSearchMethod::RISK)
		expansion_delay = std::make_unique<ExpansionDelay>(opts.get<int>("expansion_delay_window_size"));
}

void RealTimeSearch::initialize() {
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
}

SearchStatus RealTimeSearch::step() {
	++num_rts_phases;
	lc.ls->initialize(current_state);
	const auto status = lc.search();

	const SearchStatistics &current_stats = lc.ls->get_statistics();
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
		auto plan = lc.ls->get_plan();
		for (auto op_id : plan)
			solution_cost += get_adjusted_cost(task_proxy.get_operators()[op_id]);
		overall_plan.insert(std::end(overall_plan), std::begin(plan), std::end(plan));
		set_plan(overall_plan);
		return SOLVED;
	}

	if (lc.ls->get_frontier().empty())
		return FAILED;

  switch (learning_method) {
  case LearningMethod::NONE:
    break;
  case LearningMethod::DIJKSTRA:
    dijkstra_learning->apply_updates(lc.ls->get_predecessors(), lc.ls->get_frontier(), lc.ls->get_closed(), evaluate_heuristic_when_learning);
    break;
  case LearningMethod::NANCY:
    nancy_learning->apply_updates(lc.ls->get_predecessors(), lc.ls->get_frontier(), lc.ls->get_closed(), current_state);
    break;
  }

  // TODO: do it properly
  // LookaheadControl lc{nullptr, ExpansionBound(statistics, 100)};

  OperatorID best_top_level_action(0);
  switch (decision_strategy_type) {
  case DecisionStrategy::NANCY:
    assert(nancy_decision_strategy);
    best_top_level_action = nancy_decision_strategy->pick_top_level_action(lc.ls->get_search_space());
    break;
  default:
    best_top_level_action = decision_strategy->get_top_level_action(lc.ls->get_frontier(), lc.ls->get_search_space());
    break;
  }
	//const auto best_top_level_action = decision_strategy->get_top_level_action(lc.ls->get_frontier(), lc.ls->get_search_space());
	const auto parent_node = search_space.get_node(current_state);
	const auto op = task_proxy.get_operators()[best_top_level_action];
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
	lc.ls->print_statistics();
}

static auto _parse(options::OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.document_synopsis("Lazy enforced hill-climbing", "");
	parser.add_option<std::shared_ptr<Evaluator>>("h", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance_heuristic", "distance heuristic", options::OptionParser::NONE);
	parser.add_option<int>("lookahead_bound","Lookahead bound in number of expansions.", "100");
	// TODO: check if the default value here makes any sense.
	parser.add_option<double>("time_bound","Lookahead bound in milli seconds.", "1000");
	parser.add_enum_option("lookahead_search", {"A_STAR", "A_STAR_COLLECT", "F_HAT", "BREADTH_FIRST", "RISK"}, "Lookahead search algorithm", "A_STAR");
	parser.add_enum_option("lookahead_term", {"EXPANSIONS", "TIME"}, "Type of bound the algorithm is running under", "EXPANSIONS");
	parser.add_enum_option("learning", {"NONE","DIJKSTRA","NANCY"}, "What kind of learning update to perform (DIJKSTRA for heuristic values, NANCY for beliefs)", "NONE");
	parser.add_enum_option("decision_strategy", {"MINIMIN", "BELLMAN", "NANCY", "CSERNA", "K_BEST"}, "Top-level action selection strategy", "MINIMIN");
	parser.add_enum_option("feature_kind", {"JUST_H", "WITH_PARENT_H"}, "Kind of features to look up the beliefs in the data (the data format has to match)", "JUST_H");
	parser.add_enum_option("post_feature_kind", {"JUST_H", "WITH_PARENT_H"}, "Kind of features to look up the post beliefs in the data (the data format has to match)", "JUST_H");
	parser.add_option<int>("k", "Value for k-best decision strategy", "3");
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
