#include "rt_solve_all.h"

#include "../evaluator.h"
#include "../evaluation_context.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../evaluators/weighted_evaluator.h"
#include "../open_list_factory.h"
#include "../open_lists/tiebreaking_open_list.h"
#include "../options/plugin.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"

#include <cassert>
#include <memory>

namespace real_time
{

RtSolveAll::RtSolveAll(const options::Options &opts)
	:SearchEngine(opts),
	 gatherer(opts.get<std::shared_ptr<SearchEngine> >("lookahead")),
	 initial_state(state_registry.get_initial_state()),
	 //search_space(std::make_unique(state_registry)),
	 f_evaluator(opts.get<std::shared_ptr<Evaluator> >("f_eval", nullptr)),
	 evaluator(opts.get<std::shared_ptr<Evaluator> >("eval")),
	 reopen_closed_nodes(opts.get<bool>("reopen_closed_nodes")),
	 hstar_file(opts.get<std::string>("hstar_file")),
	 successors_file(opts.get<std::string>("successors_file")),
	 collect_parent_h(opts.get<bool>("collect_parent_h")),
	 reserved_time(opts.get<double>("reserved_time")),
	 timer(std::numeric_limits<double>::infinity())
{
}

void RtSolveAll::initialize()
{
	gatherer->search();
	expanded_states = gatherer->get_expanded_states();
	state_registry = std::move(gatherer->state_registry);
	assert(gatherer->get_status() == SOLVED);
	set_plan(gatherer->get_plan());

	assert(expanded_states != nullptr);
	assert(expanded_states->size() > 0);

	// gatherer is a unique_ptr, setting it to null calls its
	// destructor.
	gatherer = nullptr;
	assert(expanded_states != nullptr);
	std::cout << "Finished initial search, continuing to solve " << expanded_states->size() << " states\n";
	assert(expanded_states->size() > 0);
	std::vector<std::shared_ptr<Evaluator>> evals{f_evaluator, evaluator};
	Options options;
	options.set("evals", evals);
	options.set("pref_only", false);
	options.set("unsafe_pruning", false);
	open_list = std::make_unique<tiebreaking_open_list::TieBreakingOpenListFactory>(options)->create_state_open_list();
	init_next_open_list();
	assert(search_space != nullptr);
}

void RtSolveAll::dump_hstar_values() const
{
	auto out = std::ofstream(hstar_file);
	if (collect_parent_h) {
		for (const auto &[state_id, values] : solved_states) {
			const auto [h, hstar, ph] = values;
			out << h << ' ' << hstar << ' ' << ph << '\n';
		}
	} else {
		for (const auto &[state_id, values] : solved_states) {
			const auto [h, hstar, ph] = values;
			out << h << ' ' << hstar << '\n';
		}
	}

	std::cout << "wrote h* data to " << hstar_file << '\n';
}

void RtSolveAll::dump_succ_value(std::ostream &out, const GlobalState &state, int h)
{
	applicables.clear();
	successor_generator.generate_applicable_ops(state, applicables);
	assert(!applicables.empty());
	out << h;

	for (auto op_id : applicables) {
		auto op = task_proxy.get_operators()[op_id];
		auto successor = state_registry.get_successor_state(state, op);
		auto eval_context = EvaluationContext(successor);
		if (!eval_context.is_evaluator_value_infinite(evaluator.get()))
			out << " " << get_adjusted_cost(op) << " " << eval_context.get_evaluator_value(evaluator.get());
	}

	out << '\n';
}

void RtSolveAll::dump_succ_values()
{
	auto out = std::ofstream(successors_file);
	for (const auto &[state_id, values] : solved_states) {
		const auto [h, hstar, ph] = values;
		GlobalState const &state{state_registry.lookup_state(state_id)};
		if (task_properties::is_goal_state(task_proxy, state) || hstar == EvaluationResult::INFTY)
			continue;
		dump_succ_value(out, state, h);
	}
	for (const auto &state_id : *expanded_states) {
		GlobalState const &state{state_registry.lookup_state(state_id)};
		EvaluationContext evc{state};
		if (task_properties::is_goal_state(task_proxy, state) || evc.is_evaluator_value_infinite(evaluator.get()))
			continue;
		const auto h = evc.get_evaluator_value(evaluator.get());
		dump_succ_value(out, state, h);
	}

	std::cout << "wrote successors data to " << successors_file << '\n';
}

void RtSolveAll::init_next_open_list()
{
	assert(open_list != nullptr);
	auto const next_id = *(expanded_states->begin());
	auto const next_state = state_registry.lookup_state(next_id);
	open_list->clear();
	search_space = std::make_unique<SearchSpace>(state_registry);
	auto initial_node = search_space->get_node(next_state);
	initial_node.open_initial();
	EvaluationContext evc{next_state, 0, true, &statistics};
	assert(!evc.is_evaluator_value_infinite(evaluator.get()));
	open_list->insert(evc, next_id);
}

SearchStatus RtSolveAll::update_hstar(SearchNode const &node, int hstar)
{
	auto cur_state = node.get_state();
	int h, ph;

	for (;;) {
		const auto exp_it = expanded_states->find(cur_state.get_id());
		auto cur_node = search_space->get_node(cur_state);
		const bool has_parent = cur_node.get_creating_operator() != OperatorID::no_operator;
		if (exp_it != std::end(*expanded_states)) {
			expanded_states->erase(exp_it);
			EvaluationContext evc{cur_state};
			h = evc.get_evaluator_value(evaluator.get());
			if (has_parent) {
				auto const parent = state_registry.lookup_state(cur_node.get_parent_state_id());
				EvaluationContext pevc{parent};
				ph = pevc.get_evaluator_value(evaluator.get());
			} else {
				ph = h;
			}
			assert(solved_states.find(cur_state.get_id()) == std::end(solved_states));
			assert(evc.get_evaluator_value(evaluator.get()) <= hstar);
			solved_states.emplace(cur_state.get_id(), std::make_tuple(h, hstar, ph));
		}

		if (!has_parent)
			break;

		cur_state = state_registry.lookup_state(cur_node.get_parent_state_id());
		hstar += get_adjusted_cost(task_proxy.get_operators()[cur_node.get_creating_operator()]);
	}

	if (expanded_states->empty()) {
		dump_hstar_values();
		dump_succ_values();
		return SOLVED;
	}

	init_next_open_list();

	return IN_PROGRESS;
}

std::tuple<GlobalState, SearchNode, bool> RtSolveAll::fetch_next_node()
{
	assert(open_list != nullptr);
	while (true) {
		if (open_list->empty()) {
			std::cout << "Completely explored state space -- no solution!\n";
			const GlobalState &initial_state = state_registry.get_initial_state();
			SearchNode dummy_node = search_space->get_node(initial_state);
			return std::make_tuple(initial_state, dummy_node, false);
		}
		StateID id = open_list->remove_min();
		GlobalState s = state_registry.lookup_state(id);
		SearchNode node = search_space->get_node(s);

		if (node.is_closed())
			continue;

		assert(!node.is_dead_end());

		node.close();
		assert(!node.is_dead_end());
		// update_f_value_statistics(node);
		statistics.inc_expanded();
		return std::make_tuple(s, node, true);
	}
}

void RtSolveAll::eval_node(SearchNode &sn, OperatorProxy const &op, SearchNode const &n)
{
	int const adj_cost = get_adjusted_cost(op);
	if (sn.is_new()) {
		int sg = n.get_g() + adj_cost;
		EvaluationContext succ_eval_context{sn.get_state(), sg, false, &statistics};
		statistics.inc_evaluated_states();

		if (open_list->is_dead_end(succ_eval_context)) {
			sn.mark_as_dead_end();
			statistics.inc_dead_ends();
			return;
		}

		sn.open(n, op, adj_cost);
		open_list->insert(succ_eval_context, sn.get_state_id());
		// don't care about checking search progress or
		// preferred ops here
	} else if (sn.get_g() > n.get_g() + adj_cost) {
		if (reopen_closed_nodes) {
			if (sn.is_closed())
				statistics.inc_reopened();
			sn.reopen(n, op, adj_cost);
			EvaluationContext succ_eval_context{sn.get_state(), sn.get_g(), false, &statistics};
			open_list->insert(succ_eval_context, sn.get_state_id());
		} else {
			sn.update_parent(n, op, adj_cost);
		}
	}
}

SearchStatus RtSolveAll::step()
{
	if (timer.is_expired()) {
		dump_hstar_values();
		dump_succ_values();
		return TIMEOUT;
	}

	auto next = fetch_next_node();
	auto const &s = std::get<0>(next);
	auto &n = std::get<1>(next);
	auto const b = std::get<2>(next);
	if (!b)
		return FAILED;

	if (task_properties::is_goal_state(task_proxy, s)) {
		return update_hstar(n, 0);
	}

	applicables.clear();
	successor_generator.generate_applicable_ops(s, applicables);

	// no pruning method here
	// no preferred ops here

	for (auto op_id : applicables) {
		OperatorProxy op = task_proxy.get_operators()[op_id];
		if ((n.get_real_g() + op.get_cost()) >= bound)
			continue;

		auto succ_state = state_registry.get_successor_state(s, op);
		statistics.inc_generated();
		auto succ_node = search_space->get_node(succ_state);
		if (succ_node.is_dead_end())
			continue;

		eval_node(succ_node, op, n);
	}

	return IN_PROGRESS;
}

void RtSolveAll::print_statistics() const
{
	statistics.print_detailed_statistics();
	std::cout << "Number of solved states: " << solved_states.size() << "\n";
}

static std::shared_ptr<SearchEngine> _parse(options::OptionParser &parser) {
	parser.document_synopsis("RT Data Generation search","");
	parser.add_option<std::shared_ptr<SearchEngine>>("lookahead", "kind of lookahead search to run (currently only real_time searches are allowed for this");
	parser.add_option<std::shared_ptr<Evaluator>>("eval", "evaluator for h-value");
	parser.add_option<std::string>("hstar_file", "file name to dump h* values", "hstar_values.txt");
	parser.add_option<std::string>("successors_file", "file name to dump post-expansion data", "successors_data.txt");
	parser.add_option<double>("reserved_time", "reserved time to dump data", "0", Bounds("0", ""));
	parser.add_option<bool>("collect_parent_h", "also collect and print the parent h for each state", "false");
	parser.add_option<bool>("reopen_closed_nodes", "reopen closed nodes when solving", "true");

	SearchEngine::add_pruning_option(parser);
	SearchEngine::add_options_to_parser(parser);
	Options opts = parser.parse();

	std::shared_ptr<RtSolveAll> engine;
	if (!parser.dry_run()) {
		auto g = std::make_shared<g_evaluator::GEvaluator>();
		auto h = opts.get<std::shared_ptr<Evaluator>>("eval");
		auto f = std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>({g, h}));
		Options open_list_options;
		open_list_options.set("evals", std::vector<std::shared_ptr<Evaluator>>{f, h});
		open_list_options.set("pref_only", false);
		open_list_options.set("unsafe_pruning", false);
		opts.set<std::shared_ptr<OpenListFactory>>("open", std::make_shared<tiebreaking_open_list::TieBreakingOpenListFactory>(open_list_options));
		opts.set<std::shared_ptr<Evaluator>>("f_eval", f);
		opts.set("reopen_closed", true);
		std::vector<std::shared_ptr<Evaluator>> preferred_list;
		opts.set("preferred", preferred_list);
		engine = std::make_shared<RtSolveAll>(opts);
	}

	return engine;
}

static options::Plugin<SearchEngine> _plugin("rt_solve_all", _parse);
}
