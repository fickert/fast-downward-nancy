#include "real_time_search.h"

#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../plugin.h"
#include "../task_utils/task_properties.h"
#include "../utils/system.h"
#include "util.h"
#include "DiscreteDistribution.h"
#include "risk_search.h"

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
	if (opts.contains("hstar_data"))
		hstar_data = std::make_unique<hstar_data_type<int>>(read_hstar_data<int>(opts.get<std::string>("hstar_data")));
	if (opts.contains("post_expansion_belief_data"))
		post_expansion_belief_data = std::make_unique<hstar_data_type<long long>>(read_hstar_data<long long>(opts.get<std::string>("post_expansion_belief_data")));

	heuristic = opts.get<std::shared_ptr<Evaluator>>("h");
	base_heuristic = heuristic;
	if (learning_method == LearningMethod::DIJKSTRA)
		heuristic = std::make_shared<LearningEvaluator>(heuristic);
	initialize_optional_features(opts);

  switch (learning_method) {
  case LearningMethod::NONE:
    dijkstra_learning = nullptr;
    break;
  case LearningMethod::DIJKSTRA:
    dijkstra_learning = std::make_unique<DijkstraLearning>(std::static_pointer_cast<LearningEvaluator>(heuristic),
                                                           std::static_pointer_cast<LearningEvaluator>(distance_heuristic),
                                                           state_registry);
    break;
  case LearningMethod::NANCY:
    const StateRegistry &csr = state_registry;
    nancy_learning = std::make_unique<NancyLearning>(csr);
    break;
  }
  bool const store_exploration_data = learning_method != LearningMethod::NONE;

	switch (LookaheadSearchMethod(opts.get_enum("lookahead_search"))) {
	case LookaheadSearchMethod::A_STAR:
		lookahead_search = std::make_unique<AStarLookaheadSearch>(state_registry, opts.get<int>("lookahead_bound"), heuristic, store_exploration_data, expansion_delay.get(), heuristic_error.get());
		break;
	case LookaheadSearchMethod::BREADTH_FIRST:
		lookahead_search = std::make_unique<BreadthFirstLookaheadSearch>(state_registry, opts.get<int>("lookahead_bound"), store_exploration_data, expansion_delay.get(), heuristic_error.get());
		break;
	case LookaheadSearchMethod::F_HAT:
		lookahead_search = std::make_unique<FHatLookaheadSearch>(state_registry, opts.get<int>("lookahead_bound"), heuristic, distance_heuristic, store_exploration_data, expansion_delay.get(), *heuristic_error);
		break;
	case LookaheadSearchMethod::RISK:
		lookahead_search = std::make_unique<RiskLookaheadSearch>(state_registry, opts.get<int>("lookahead_bound"), heuristic, base_heuristic, distance_heuristic, store_exploration_data, expansion_delay.get(), heuristic_error.get(), hstar_data.get(), post_expansion_belief_data.get());
		break;
	default:
		std::cerr << "unknown lookahead search method: " << opts.get_enum("lookahead_search") << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
	}

	switch (DecisionStrategy(opts.get_enum("decision_strategy"))) {
	case DecisionStrategy::MINIMIN: {
		auto f_evaluator = std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{heuristic, std::make_shared<g_evaluator::GEvaluator>()});
		decision_strategy = std::make_unique<ScalarDecisionStrategy>(state_registry, [this, f_evaluator](const StateID &state_id, SearchSpace &lookahead_search_space) {
			const auto state = state_registry.lookup_state(state_id);
			const auto node = lookahead_search_space.get_node(state);
			auto eval_context = EvaluationContext(state, node.get_g(), false, nullptr, false);
			return eval_context.get_evaluator_value_or_infinity(f_evaluator.get());
		});
		break;
	}
	case DecisionStrategy::BELLMAN: {
		auto f_hat_evaluator = create_f_hat_evaluator(heuristic, distance_heuristic, *heuristic_error);
		decision_strategy = std::make_unique<ScalarDecisionStrategy>(state_registry, [this, f_hat_evaluator](const StateID &state_id, SearchSpace &lookahead_search_space) {
			const auto state = state_registry.lookup_state(state_id);
			const auto node = lookahead_search_space.get_node(state);
			auto eval_context = EvaluationContext(state, node.get_g(), false, nullptr, false);
			return eval_context.get_evaluator_value_or_infinity(f_hat_evaluator.get());
		});
		break;
	}
	case DecisionStrategy::NANCY: {
    auto beliefs = this->lookahead_search->get_beliefs();
    assert(beliefs);
		decision_strategy = std::make_unique<ProbabilisticDecisionStrategy>(state_registry, beliefs);
		break;
	}
	default:
		std::cerr << "unknown decision strategy: " << opts.get_enum("decision_strategy") << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
	}

	heuristic->notify_initial_state(current_state);
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
	lookahead_search->initialize(current_state);
	const auto status = lookahead_search->search();

	const SearchStatistics &current_stats = lookahead_search->get_statistics();
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
		auto plan = lookahead_search->get_plan();
		for (auto op_id : plan)
			solution_cost += task_proxy.get_operators()[op_id].get_cost();
		overall_plan.insert(std::end(overall_plan), std::begin(plan), std::end(plan));
		set_plan(overall_plan);
		return SOLVED;
	}

	if (lookahead_search->get_frontier().empty())
		return FAILED;

  switch (learning_method) {
  case LearningMethod::NONE:
    break;
  case LearningMethod::DIJKSTRA:
    dijkstra_learning->apply_updates(lookahead_search->get_predecessors(), lookahead_search->get_frontier(), lookahead_search->get_closed(), evaluate_heuristic_when_learning);
    break;
  case LearningMethod::NANCY:
    {
      auto beliefs = lookahead_search->get_beliefs();
      assert(beliefs != nullptr);
      nancy_learning->apply_updates(lookahead_search->get_predecessors(), lookahead_search->get_frontier(), lookahead_search->get_closed(), beliefs);
      // TODO: implement
    }
    break;
  }

	const auto best_top_level_action = decision_strategy->get_top_level_action(lookahead_search->get_frontier(), lookahead_search->get_search_space());
	const auto parent_node = search_space.get_node(current_state);
	const auto op = task_proxy.get_operators()[best_top_level_action];
	solution_cost += op.get_cost();
	current_state = state_registry.get_successor_state(current_state, op);
	auto next_node = search_space.get_node(current_state);
	if (next_node.is_new())
    next_node.open(parent_node, op, op.get_cost());
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
	lookahead_search->print_statistics();
}

static auto _parse(options::OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.document_synopsis("Lazy enforced hill-climbing", "");
	parser.add_option<std::shared_ptr<Evaluator>>("h", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance_heuristic", "distance heuristic", options::OptionParser::NONE);
	parser.add_option<int>("lookahead_bound","Lookahead bound in number of expansions.", "100");
	parser.add_enum_option("lookahead_search", {"A_STAR", "F_HAT", "BREADTH_FIRST", "RISK"}, "Lookahead search algorithm", "A_STAR");
	parser.add_enum_option("learning", {"NONE","DIJKSTRA","NANCY"}, "What kind of learning update to perform (DIJKSTRA for heuristic values, NANCY for beliefs)", "NONE");
	parser.add_enum_option("decision_strategy", {"MINIMIN", "BELLMAN", "NANCY", "CSERNA", "K_BEST"}, "Top-level action selection strategy", "MINIMIN");
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
