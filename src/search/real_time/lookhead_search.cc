#include "lookhead_search.h"

#include "debiased_heuristic.h"
#include "expansion_delay.h"
#include "heuristic_error.h"
#include "util.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../open_lists/tiebreaking_open_list.h"
#include "../options/plugin.h"
#include "../tasks/root_task.h"
#include "../task_utils/task_properties.h"
#include "../open_lists/best_first_open_list.h"

namespace real_time {

void LookaheadSearch::mark_expanded(SearchNode &node) {
	statistics->inc_expanded();
	node.close();
	if (store_exploration_data)
		closed.emplace(node.get_state_id());
	if (expansion_delay)
		expansion_delay->update_expansion_delay(statistics->get_expanded() - open_list_insertion_time[node.get_state_id()]);
}

auto LookaheadSearch::check_goal_and_set_plan(const GlobalState &state) -> bool {
	if (task_properties::is_goal_state(task_proxy, state)) {
		auto plan = Plan();
		search_space->trace_path(state, plan);
		this->plan = plan;
		return true;
	}
	return false;
}

LookaheadSearch::LookaheadSearch(StateRegistry &state_registry, int lookahead_bound, bool store_exploration_data, ExpansionDelay *expansion_delay, HeuristicError *heuristic_error) :
	solution_found(false),
	task(tasks::g_root_task),
	task_proxy(*task),
	state_registry(state_registry),
	successor_generator(successor_generator::g_successor_generators[task_proxy]),
	lookahead_bound(lookahead_bound),
	store_exploration_data(store_exploration_data),
	expansion_delay(expansion_delay),
	heuristic_error(heuristic_error) {}

void LookaheadSearch::initialize(const GlobalState &initial_state) {
	solution_found = false;
	plan.clear();
	search_space = std::make_unique<SearchSpace>(state_registry);
	statistics = std::make_unique<SearchStatistics>();
	auto node = search_space->get_node(initial_state);
	node.open_initial();
	if (store_exploration_data) {
		predecessors.clear();
		frontier.clear();
		closed.clear();
		closed.reserve(lookahead_bound);
	}
	if (expansion_delay)
		open_list_insertion_time.clear();
}

EagerLookaheadSearch::EagerLookaheadSearch(StateRegistry &state_registry, int lookahead_bound, bool store_exploration_data, ExpansionDelay *expansion_delay, HeuristicError *heuristic_error) :
	LookaheadSearch(state_registry, lookahead_bound, store_exploration_data, expansion_delay, heuristic_error) {}

void EagerLookaheadSearch::initialize(const GlobalState &initial_state) {
	LookaheadSearch::initialize(initial_state);
	open_list = create_open_list();
	auto eval_context = EvaluationContext(initial_state, 0, false, statistics.get());
	statistics->inc_evaluated_states();
	open_list->insert(eval_context, initial_state.get_id());
	if (expansion_delay)
		open_list_insertion_time[initial_state.get_id()] = 0;
}

auto EagerLookaheadSearch::search() -> SearchStatus {
	assert(statistics);
	while (statistics->get_expanded() < lookahead_bound) {
		if (open_list->empty())
			return FAILED;

		const auto id = open_list->remove_min();
		const auto state = state_registry.lookup_state(id);
		auto node = search_space->get_node(state);

		if (node.is_closed())
			continue;

		mark_expanded(node);

		if (check_goal_and_set_plan(state))
			return SOLVED;

		auto applicable_ops = std::vector<OperatorID>();
		successor_generator.generate_applicable_ops(state, applicable_ops);

		auto eval_context = EvaluationContext(state, node.get_g(), false, statistics.get());
		if (heuristic_error)
			heuristic_error->set_expanding_state(state);

		for (auto op_id : applicable_ops) {
			const auto op = task_proxy.get_operators()[op_id];
			const auto succ_state = state_registry.get_successor_state(state, op);
			statistics->inc_generated();
			auto succ_node = search_space->get_node(succ_state);

			if (store_exploration_data)
				predecessors[succ_state.get_id()].emplace_back(id, op);

			// Previously encountered dead end. Don't re-evaluate.
			if (succ_node.is_dead_end())
				continue;

			if (succ_node.is_new()) {
				auto succ_eval_context = EvaluationContext(succ_state, node.get_g() + get_adjusted_cost(op), false, statistics.get());
				statistics->inc_evaluated_states();
				if (open_list->is_dead_end(succ_eval_context)) {
					succ_node.mark_as_dead_end();
					statistics->inc_dead_ends();
					continue;
				}
				succ_node.open(node, op, get_adjusted_cost(op)));
				open_list->insert(succ_eval_context, succ_state.get_id());
			} else if (succ_node.get_g() > node.get_g() get_adjusted_cost(op)) {
				// We found a new cheapest path to an open or closed state.
				if (succ_node.is_closed())
					statistics->inc_reopened();
				succ_node.reopen(node, op, get_adjusted_cost(op));
				auto succ_eval_context = EvaluationContext(succ_state, succ_node.get_g(), false, statistics.get());
				open_list->insert(succ_eval_context, succ_state.get_id());
			}
			open_list_insertion_time[id] = statistics->get_expanded();
			if (heuristic_error)
				heuristic_error->add_successor(succ_node, get_adjusted_cost(op));
		}
		if (heuristic_error)
			heuristic_error->update_error();
	}
	while (!open_list->empty())
		frontier.push_back(open_list->remove_min());
	return IN_PROGRESS;
}

auto AStarLookaheadSearch::create_open_list() const -> std::unique_ptr<StateOpenList> {
	auto options = options::Options();
	options.set("evals", std::vector<std::shared_ptr<Evaluator>>{f_evaluator, heuristic});
	options.set("pref_only", false);
	options.set("unsafe_pruning", false);
	return std::make_unique<tiebreaking_open_list::TieBreakingOpenListFactory>(options)->create_state_open_list();
}

AStarLookaheadSearch::AStarLookaheadSearch(StateRegistry &state_registry, int lookahead_bound, std::shared_ptr<Evaluator> heuristic, bool store_exploration_data, ExpansionDelay *expansion_delay, HeuristicError *heuristic_error) :
	EagerLookaheadSearch(state_registry, lookahead_bound, store_exploration_data, expansion_delay, heuristic_error),
	f_evaluator(std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{heuristic, std::make_shared<g_evaluator::GEvaluator>()})),
	heuristic(heuristic) {}

auto BreadthFirstLookaheadSearch::create_open_list() const -> std::unique_ptr<StateOpenList> {
	auto options = options::Options();
	options.set<std::shared_ptr<Evaluator>>("eval", std::make_shared<g_evaluator::GEvaluator>());
	options.set("pref_only", false);
	return std::make_unique<standard_scalar_open_list::BestFirstOpenListFactory>(options)->create_state_open_list();
}

BreadthFirstLookaheadSearch::BreadthFirstLookaheadSearch(StateRegistry &state_registry, int lookahead_bound, bool store_exploration_data, ExpansionDelay *expansion_delay, HeuristicError *heuristic_error) :
	EagerLookaheadSearch(state_registry, lookahead_bound, store_exploration_data, expansion_delay, heuristic_error) {}

auto FHatLookaheadSearch::create_open_list() const -> std::unique_ptr<StateOpenList> {
	auto options = options::Options();
	options.set("evals", std::vector<std::shared_ptr<Evaluator>>{f_hat_evaluator, heuristic});
	options.set("pref_only", false);
	options.set("unsafe_pruning", false);
	return std::make_unique<tiebreaking_open_list::TieBreakingOpenListFactory>(options)->create_state_open_list();
}

FHatLookaheadSearch::FHatLookaheadSearch(StateRegistry &state_registry, int lookahead_bound, std::shared_ptr<Evaluator> heuristic, std::shared_ptr<Evaluator> distance, bool store_exploration_data, ExpansionDelay *expansion_delay, HeuristicError &heuristic_error) :
	EagerLookaheadSearch(state_registry, lookahead_bound, store_exploration_data, expansion_delay, &heuristic_error),
	f_hat_evaluator(create_f_hat_evaluator(heuristic, distance, heuristic_error)),
	heuristic(heuristic) {}

static options::PluginTypePlugin<LookaheadSearch> _type_plugin("LookaheadSearch", "Lookahead search engine for real-time search");

}
