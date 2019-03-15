#include "real_time_search.h"

#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../plugin.h"
#include "../utils/system.h"
#include "../task_utils/task_properties.h"

namespace real_time {

RealTimeSearch::RealTimeSearch(const options::Options &opts)
	: SearchEngine(opts),
	  current_state(state_registry.get_initial_state()),
	  num_rts_phases(0),
	  evaluate_heuristic_when_learning(LookaheadSearchMethod(opts.get_enum("lookahead_search")) == LookaheadSearchMethod::BREADTH_FIRST) {

	heuristic = opts.get<std::shared_ptr<Evaluator>>("h");
	if (opts.get<bool>("learning"))
		heuristic = std::make_shared<LearningEvaluator>(heuristic);
	initialize_f_hat_functions_if_required(opts);
	if (opts.get<bool>("learning"))
		dijkstra_learning = std::make_unique<DijkstraLearning>(std::static_pointer_cast<LearningEvaluator>(heuristic), std::static_pointer_cast<LearningEvaluator>(distance_heuristic), state_registry);

	switch (LookaheadSearchMethod(opts.get_enum("lookahead_search"))) {
	case LookaheadSearchMethod::A_STAR:
		lookahead_search = std::make_unique<AStarLookaheadSearch>(state_registry, opts.get<int>("lookahead_bound"), heuristic, static_cast<bool>(dijkstra_learning), expansion_delay.get());
		break;
	case LookaheadSearchMethod::BREADTH_FIRST:
		lookahead_search = std::make_unique<BreadthFirstLookaheadSearch>(state_registry, opts.get<int>("lookahead_bound"), static_cast<bool>(dijkstra_learning), expansion_delay.get());
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
	default:
		std::cerr << "unknown decision strategy: " << opts.get_enum("decision_strategy") << std::endl;
		utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
	}

	heuristic->notify_initial_state(current_state);
}

void RealTimeSearch::initialize_f_hat_functions_if_required(const options::Options &opts) {
	if (LookaheadSearchMethod(opts.get_enum("lookahead_search")) == LookaheadSearchMethod::F_HAT ||
		LookaheadSearchMethod(opts.get_enum("lookahead_search")) == LookaheadSearchMethod::RISK ||
		DecisionStrategy(opts.get_enum("decision_strategy")) != DecisionStrategy::MINIMIN) {
		expansion_delay = std::make_unique<ExpansionDelay>(opts.get<int>("expansion_delay_window_size"));
		if (task_properties::is_unit_cost(task_proxy) || opts.get<std::shared_ptr<Evaluator>>("h") == opts.get<std::shared_ptr<Evaluator>>("distance_heuristic")) {
			distance_heuristic = heuristic;
		} else {
			distance_heuristic = opts.get<std::shared_ptr<Evaluator>>("distance_heuristic");
			if (opts.get<bool>("learning"))
				distance_heuristic = std::make_shared<LearningEvaluator>(distance_heuristic);
		}
	}
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
		overall_plan.insert(std::end(overall_plan), std::begin(plan), std::end(plan));
		set_plan(overall_plan);
		return SOLVED;
	}

	if (dijkstra_learning)
		dijkstra_learning->apply_updates(lookahead_search->get_predecessors(), lookahead_search->get_frontier(), lookahead_search->get_closed(), evaluate_heuristic_when_learning);

	const auto best_top_level_action = decision_strategy->get_top_level_action(lookahead_search->get_frontier(), lookahead_search->get_search_space());
	const auto parent_node = search_space.get_node(current_state);
	const auto op = task_proxy.get_operators()[best_top_level_action];
	current_state = state_registry.get_successor_state(current_state, op);
	auto next_node = search_space.get_node(current_state);
	if (next_node.is_new())
		next_node.open(parent_node, op, op.get_cost());
	return IN_PROGRESS;
}

void RealTimeSearch::print_statistics() const {
	statistics.print_detailed_statistics();
	std::cout << "Number of lookahead phases: " << num_rts_phases << std::endl;
}

static auto _parse(options::OptionParser &parser) -> std::shared_ptr<SearchEngine> {
	parser.document_synopsis("Lazy enforced hill-climbing", "");
	parser.add_option<std::shared_ptr<Evaluator>>("h", "heuristic");
	parser.add_option<std::shared_ptr<Evaluator>>("distance_heuristic", "distance heuristic", options::OptionParser::NONE);
	parser.add_option<int>("lookahead_bound","Lookahead bound in number of expansions.", "100");
	parser.add_enum_option("lookahead_search", {"A_STAR", "F_HAT", "BREADTH_FIRST", "RISK"}, "Lookahead search algorithm", "A_STAR");
	parser.add_option<bool>("learning", "Update the heuristic after each lookahead search using Dijkstra", "true");
	parser.add_enum_option("decision_strategy", {"MINIMIN", "BELLMAN", "NANCY", "CSERNA", "K_BEST"}, "Top-level action selection strategy", "MINIMIN");
	parser.add_option<int>("k", "Value for k-best decision strategy", "3");
	parser.add_option<int>("expansion_delay_window_size", "Sliding average window size used for the computation of expansion delays (set this to 0 to use the global average)", "0", options::Bounds("0", ""));

	SearchEngine::add_options_to_parser(parser);
	const auto opts = parser.parse();

	if (parser.dry_run())
		return nullptr;
	return std::make_shared<RealTimeSearch>(opts);
}

static Plugin<SearchEngine> _plugin("real_time", _parse);
}
