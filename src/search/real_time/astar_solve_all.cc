#include "astar_solve_all.h"

#include "../algorithms/ordered_set.h"
#include "../evaluator.h"
#include "../evaluation_context.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../evaluators/weighted_evaluator.h"
#include "../open_list_factory.h"
#include "../open_lists/tiebreaking_open_list.h"
#include "../options/plugin.h"
#include "../pruning_method.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"

#include <cassert>
#include <fstream>
#include <memory>
#include <set>

using namespace std;

namespace astar_solve_all {
AStarSolveAll::AStarSolveAll(const Options &opts)
    : SearchEngine(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      open_list(opts.get<shared_ptr<OpenListFactory>>("open")->
                create_state_open_list()),
      current_initial_state(state_registry.get_initial_state()),
      current_search_space(std::make_unique<SearchSpace>(state_registry)),
      f_evaluator(opts.get<shared_ptr<Evaluator>>("f_eval", nullptr)),
      evaluator(opts.get<std::shared_ptr<Evaluator>>("eval")),
      weight(opts.get<int>("w")),
      preferred_operator_evaluators(opts.get_list<shared_ptr<Evaluator>>("preferred")),
      pruning_method(opts.get<shared_ptr<PruningMethod>>("pruning")),
      hstar_file(opts.get<std::string>("hstar_file")),
      successors_file(opts.get<std::string>("successors_file")),
      find_early_solutions(opts.get<bool>("find_early_solutions")),
      collect_parent_h(opts.get<bool>("collect_parent_h")),
      best_solution_state(StateID::no_state),
      best_solution_cost(std::numeric_limits<int>::max()),
      computing_initial_solution(true),
      reserved_time(opts.get<double>("reserved_time")),
      timer(std::numeric_limits<double>::infinity()) {
}

void AStarSolveAll::initialize() {
    cout << "Conducting best first search"
         << (reopen_closed_nodes ? " with" : " without")
         << " reopening closed nodes, (real) bound = " << bound
         << endl;
    assert(open_list);

    set<Evaluator *> evals;
    open_list->get_path_dependent_evaluators(evals);

    /*
      Collect path-dependent evaluators that are used for preferred operators
      (in case they are not also used in the open list).
    */
    for (const shared_ptr<Evaluator> &evaluator : preferred_operator_evaluators) {
        evaluator->get_path_dependent_evaluators(evals);
    }

    /*
      Collect path-dependent evaluators that are used in the f_evaluator.
      They are usually also used in the open list and will hence already be
      included, but we want to be sure.
    */
    if (f_evaluator) {
        f_evaluator->get_path_dependent_evaluators(evals);
    }

    path_dependent_evaluators.assign(evals.begin(), evals.end());

    const GlobalState &initial_state = state_registry.get_initial_state();
    for (Evaluator *evaluator : path_dependent_evaluators) {
        evaluator->notify_initial_state(initial_state);
    }

    /*
      Note: we consider the initial state as reached by a preferred
      operator.
    */
    EvaluationContext eval_context(initial_state, 0, true, &statistics);

    statistics.inc_evaluated_states();

    if (open_list->is_dead_end(eval_context)) {
        cout << "Initial state is a dead end." << endl;
    } else {
        if (search_progress.check_progress(eval_context))
            print_checkpoint_line(0);
        start_f_value_statistics(eval_context);
        SearchNode node = current_search_space->get_node(initial_state);
        node.open_initial();

        open_list->insert(eval_context, initial_state.get_id());
    }

    print_initial_evaluator_values(eval_context);

    pruning_method->initialize(task);

	if (!std::isinf(max_time)) {
		assert(max_time - reserved_time > 0);
		timer = utils::CountdownTimer(max_time - reserved_time);
	}
}

void AStarSolveAll::print_checkpoint_line(int g) const {
    cout << "[g=" << g << ", ";
    statistics.print_basic_statistics();
    cout << "]" << endl;
}

void AStarSolveAll::print_statistics() const {
    statistics.print_detailed_statistics();
    search_space.print_statistics();
    pruning_method->print_statistics();
}

SearchStatus AStarSolveAll::step() {
	if (timer.is_expired()) {
		dump_hstar_values();
		compute_and_dump_successors_data();
		return TIMEOUT;
	}

    pair<SearchNode, bool> n = fetch_next_node();
    if (!n.second) {
        return FAILED;
    }
    SearchNode node = n.first;

    GlobalState s = node.get_state();
	if (task_properties::is_goal_state(task_proxy, s)) {
		if (computing_initial_solution) {
			assert(solved_states.empty());
			current_search_space->trace_path(s, initial_plan);
			std::cout << "Found initial solution after " << statistics.get_expanded() << " expansions, continuing to solve all expanded states..." << std::endl;
			vector<shared_ptr<Evaluator>> evals = {f_evaluator, evaluator};
			Options options;
			options.set("evals", evals);
			options.set("pref_only", false);
			options.set("unsafe_pruning", false);
			open_list = std::make_unique<tiebreaking_open_list::TieBreakingOpenListFactory>(options)->create_state_open_list();
		}
		return update_hstar_from_state(node, 0);
	}

	if (!computing_initial_solution && find_early_solutions) {
		const auto solved_states_it = solved_states.find(s.get_id());
		if (solved_states_it != std::end(solved_states)) {
			const auto solution_cost = node.get_g() + solved_states_it->second.second;
			if (best_solution_state == StateID::no_state || solution_cost <= best_solution_cost) {
				best_solution_cost = solution_cost;
				best_solution_state = s.get_id();
			}
		}
	}

    vector<OperatorID> applicable_ops;
    successor_generator.generate_applicable_ops(s, applicable_ops);

    /*
      TODO: When preferred operators are in use, a preferred operator will be
      considered by the preferred operator queues even when it is pruned.
    */
    pruning_method->prune_operators(s, applicable_ops);

    // This evaluates the expanded state (again) to get preferred ops
    EvaluationContext eval_context(s, node.get_g(), false, &statistics, true);
    ordered_set::OrderedSet<OperatorID> preferred_operators;
    for (const shared_ptr<Evaluator> &preferred_operator_evaluator : preferred_operator_evaluators) {
        collect_preferred_operators(eval_context,
                                    preferred_operator_evaluator.get(),
                                    preferred_operators);
    }

	if (!computing_initial_solution
			&& find_early_solutions
			&& best_solution_state != StateID::no_state
			&& eval_context.get_evaluator_value_or_infinity(f_evaluator.get()) == best_solution_cost) {
		const auto solution_node = current_search_space->get_node(state_registry.lookup_state(best_solution_state));
		return update_hstar_from_state(solution_node, best_solution_cost - solution_node.get_g());
	}

    for (OperatorID op_id : applicable_ops) {
        OperatorProxy op = task_proxy.get_operators()[op_id];
        if ((node.get_real_g() + op.get_cost()) >= bound)
            continue;

        GlobalState succ_state = state_registry.get_successor_state(s, op);
        statistics.inc_generated();
        bool is_preferred = preferred_operators.contains(op_id);

        SearchNode succ_node = current_search_space->get_node(succ_state);

        for (Evaluator *evaluator : path_dependent_evaluators) {
            evaluator->notify_state_transition(s, op_id, succ_state);
        }

        // Previously encountered dead end. Don't re-evaluate.
        if (succ_node.is_dead_end())
            continue;

        if (succ_node.is_new()) {
            // We have not seen this state before.
            // Evaluate and create a new node.

            // Careful: succ_node.get_g() is not available here yet,
            // hence the stupid computation of succ_g.
            // TODO: Make this less fragile.
            int succ_g = node.get_g() + get_adjusted_cost(op);

            EvaluationContext succ_eval_context(
                succ_state, succ_g, is_preferred, &statistics);
            statistics.inc_evaluated_states();

            if (open_list->is_dead_end(succ_eval_context)) {
                succ_node.mark_as_dead_end();
                statistics.inc_dead_ends();
                continue;
            }
            succ_node.open(node, op, get_adjusted_cost(op));

            open_list->insert(succ_eval_context, succ_state.get_id());
            if (search_progress.check_progress(succ_eval_context)) {
                print_checkpoint_line(succ_node.get_g());
                reward_progress();
            }

            if (collect_parent_h) {
              parent.insert_or_assign(succ_state.get_id(), s.get_id());
            }
        } else if (succ_node.get_g() > node.get_g() + get_adjusted_cost(op)) {
            // We found a new cheapest path to an open or closed state.
            if (collect_parent_h) {
              parent.insert_or_assign(succ_state.get_id(), s.get_id());
            }
            if (reopen_closed_nodes) {
                if (succ_node.is_closed()) {
                    /*
                      TODO: It would be nice if we had a way to test
                      that reopening is expected behaviour, i.e., exit
                      with an error when this is something where
                      reopening should not occur (e.g. A* with a
                      consistent heuristic).
                    */
                    statistics.inc_reopened();
                }
                succ_node.reopen(node, op, get_adjusted_cost(op));

                EvaluationContext succ_eval_context(
                    succ_state, succ_node.get_g(), is_preferred, &statistics);

                /*
                  Note: our old code used to retrieve the h value from
                  the search node here. Our new code recomputes it as
                  necessary, thus avoiding the incredible ugliness of
                  the old "set_evaluator_value" approach, which also
                  did not generalize properly to settings with more
                  than one evaluator.

                  Reopening should not happen all that frequently, so
                  the performance impact of this is hopefully not that
                  large. In the medium term, we want the evaluators to
                  remember evaluator values for states themselves if
                  desired by the user, so that such recomputations
                  will just involve a look-up by the Evaluator object
                  rather than a recomputation of the evaluator value
                  from scratch.
                */
                open_list->insert(succ_eval_context, succ_state.get_id());
            } else {
                // If we do not reopen closed nodes, we just update the parent pointers.
                // Note that this could cause an incompatibility between
                // the g-value and the actual path that is traced back.
                succ_node.update_parent(node, op, get_adjusted_cost(op));
            }
        }
    }

    return IN_PROGRESS;
}

void AStarSolveAll::dump_hstar_values() const {
	auto out = std::ofstream(hstar_file);
	for (const auto &[state_id, h_values] : solved_states) {
		const auto [h, hstar] = h_values;
		if (collect_parent_h) {
			const auto p = parent.find(state_id);
			if (p == parent.end())
				continue;
			const auto ph = solved_states.find(p->second);
			if (ph == solved_states.end())
				continue;
			out << h << " " << hstar <<  " " << ph->second.first << '\n';
		} else {
			out << h << " " << hstar << '\n';
		}
	}
	out.close();
	std::cout << "dumped h* values to " << hstar_file << std::endl;
}

void AStarSolveAll::compute_and_dump_successors_data() {
	auto out = std::ofstream(successors_file);
	const auto dump_successors_data = [&out, this](const auto &state) {
		auto applicable_ops = std::vector<OperatorID>();
		successor_generator.generate_applicable_ops(state, applicable_ops);
		assert(!applicable_ops.empty());
		for (auto op_id : applicable_ops) {
			auto op = task_proxy.get_operators()[op_id];
			auto successor = state_registry.get_successor_state(state, op);
			auto eval_context = EvaluationContext(successor);
			if (!eval_context.is_evaluator_value_infinite(evaluator.get()))
				out << " " << get_adjusted_cost(op) << " " << eval_context.get_evaluator_value(evaluator.get());
		}
	};
	for (const auto &[state_id, h_values] : solved_states) {
		auto state = state_registry.lookup_state(state_id);
		const auto [h, hstar] = h_values;
		if (task_properties::is_goal_state(task_proxy, state) || hstar == EvaluationResult::INFTY)
			continue;
		out << h;
		dump_successors_data(state);
		out << '\n';
	}
	for (const auto &state_id : expanded_states) {
		auto state = state_registry.lookup_state(state_id);
		auto eval_context = EvaluationContext(state);
		if (task_properties::is_goal_state(task_proxy, state) || eval_context.is_evaluator_value_infinite(evaluator.get()))
			continue;
		const auto h = eval_context.get_evaluator_value(evaluator.get());
		out << h;
		dump_successors_data(state);
		out << '\n';
	}
	out.close();
	std::cout << "dumped successors data to " << successors_file << std::endl;
}

auto AStarSolveAll::update_hstar_from_state(const SearchNode &node, int hstar) -> SearchStatus {
	auto current_state = node.get_state();

	if (!computing_initial_solution || weight <= 1) {
		for (;;) {
			const auto expanded_states_it = expanded_states.find(current_state.get_id());
			if (expanded_states_it != std::end(expanded_states)) {
				expanded_states.erase(expanded_states_it);
				auto eval_context = EvaluationContext(current_state);
				assert(solved_states.find(current_state.get_id()) == std::end(solved_states));
				assert(eval_context.get_evaluator_value(evaluator.get()) <= hstar);
				solved_states.emplace(current_state.get_id(), std::make_pair(eval_context.get_evaluator_value(evaluator.get()), hstar));
			}

			auto current_node = current_search_space->get_node(current_state);
			if (current_node.get_creating_operator() == OperatorID::no_operator)
				break;

			current_state = state_registry.lookup_state(current_node.get_parent_state_id());
			hstar += get_adjusted_cost(task_proxy.get_operators()[current_node.get_creating_operator()]);
		}
	}

	computing_initial_solution = false;

	if (expanded_states.empty()) {
		set_plan(initial_plan);
		dump_hstar_values();
		compute_and_dump_successors_data();
		return SOLVED;
	}

	open_list->clear();
	current_search_space = std::make_unique<SearchSpace>(state_registry);
	const auto next_initial_state_id = *std::begin(expanded_states);
	const auto next_initial_state = state_registry.lookup_state(next_initial_state_id);
	auto initial_node = current_search_space->get_node(next_initial_state);
	initial_node.open_initial();
	auto eval_context = EvaluationContext(next_initial_state, 0, true, &statistics);
	assert(!eval_context.is_evaluator_value_infinite(evaluator.get()));
	open_list->insert(eval_context, next_initial_state_id);
	if (find_early_solutions) {
		best_solution_cost = std::numeric_limits<int>::max();
		best_solution_state = StateID::no_state;
	}
	return IN_PROGRESS;
}

pair<SearchNode, bool> AStarSolveAll::fetch_next_node() {
    /* TODO: The bulk of this code deals with multi-path dependence,
       which is a bit unfortunate since that is a special case that
       makes the common case look more complicated than it would need
       to be. We could refactor this by implementing multi-path
       dependence as a separate search algorithm that wraps the "usual"
       search algorithm and adds the extra processing in the desired
       places. I think this would lead to much cleaner code. */

    while (true) {
        if (open_list->empty()) {
            cout << "Completely explored state space -- no solution!" << endl;
            // HACK! HACK! we do this because SearchNode has no default/copy constructor
            const GlobalState &initial_state = state_registry.get_initial_state();
            SearchNode dummy_node = current_search_space->get_node(initial_state);
            return make_pair(dummy_node, false);
        }
        StateID id = open_list->remove_min();
        // TODO is there a way we can avoid creating the state here and then
        //      recreate it outside of this function with node.get_state()?
        //      One way would be to store GlobalState objects inside SearchNodes
        //      instead of StateIDs
        GlobalState s = state_registry.lookup_state(id);
        SearchNode node = current_search_space->get_node(s);

        if (node.is_closed())
            continue;

        assert(!node.is_dead_end());

		if (computing_initial_solution)
			expanded_states.emplace(id);
        node.close();
        assert(!node.is_dead_end());
        update_f_value_statistics(node);
        statistics.inc_expanded();
        return make_pair(node, true);
    }
}

void AStarSolveAll::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // one of the heuristics finds a state with a new best h value.
    open_list->boost_preferred();
}

void AStarSolveAll::dump_search_space() const {
    search_space.dump(task_proxy);
}

void AStarSolveAll::start_f_value_statistics(EvaluationContext &eval_context) {
    if (f_evaluator) {
        int f_value = eval_context.get_evaluator_value(f_evaluator.get());
        statistics.report_f_value_progress(f_value);
    }
}

/* TODO: HACK! This is very inefficient for simply looking up an h value.
   Also, if h values are not saved it would recompute h for each and every state. */
void AStarSolveAll::update_f_value_statistics(const SearchNode &node) {
    if (f_evaluator) {
        /*
          TODO: This code doesn't fit the idea of supporting
          an arbitrary f evaluator.
        */
        EvaluationContext eval_context(node.get_state(), node.get_g(), false, &statistics);
        int f_value = eval_context.get_evaluator_value(f_evaluator.get());
        statistics.report_f_value_progress(f_value);
    }
}

static shared_ptr<SearchEngine> _parse(options::OptionParser &parser) {
	parser.document_synopsis(
		"A* search (eager)",
		"A* is a special case of eager best first search that uses g+h "
		"as f-function. "
		"We break ties using the evaluator. Closed nodes are re-opened.");
	parser.document_note(
		"Equivalent statements using general eager search",
		"\n```\n--search astar(evaluator)\n```\n"
		"is equivalent to\n"
		"```\n--evaluator h=evaluator\n"
		"--search eager(tiebreaking([sum([g(), h]), h], unsafe_pruning=false),\n"
		"               reopen_closed=true, f_eval=sum([g(), h]))\n"
		"```\n", true);
	parser.add_option<shared_ptr<Evaluator>>("eval", "evaluator for h-value");
	parser.add_option<int>("w", "evaluator weight", "1");
	parser.add_option<std::string>("hstar_file", "file name to dump h* values", "hstar_values.txt");
	parser.add_option<std::string>("successors_file", "file name to dump post-expansion data", "successors_data.txt");
	parser.add_option<double>("reserved_time", "reserved time to dump data", "0", Bounds("0", ""));
	parser.add_option<bool>("find_early_solutions", "stop when finding an optimal solution through a node with known h* value", "false");
	parser.add_option<bool>("collect_parent_h", "also collect and print the parent h for each state", "false");

	SearchEngine::add_pruning_option(parser);
	SearchEngine::add_options_to_parser(parser);
	Options opts = parser.parse();

	shared_ptr<AStarSolveAll> engine;
	if (!parser.dry_run()) {
		auto g = make_shared<g_evaluator::GEvaluator>();
		auto h = opts.get<shared_ptr<Evaluator>>("eval");
		auto weighted_h = make_shared<weighted_evaluator::WeightedEvaluator>(h, opts.get<int>("w"));
		auto f = make_shared<sum_evaluator::SumEvaluator>(vector<shared_ptr<Evaluator>>({g, h}));
		auto weighted_f = make_shared<sum_evaluator::SumEvaluator>(vector<shared_ptr<Evaluator>>({g, weighted_h}));
		Options open_list_options;
		open_list_options.set("evals", vector<shared_ptr<Evaluator>>{weighted_f, h});
		open_list_options.set("pref_only", false);
		open_list_options.set("unsafe_pruning", false);
		opts.set<std::shared_ptr<OpenListFactory>>("open", make_shared<tiebreaking_open_list::TieBreakingOpenListFactory>(open_list_options));
		opts.set<std::shared_ptr<Evaluator>>("f_eval", f);
		opts.set("reopen_closed", true);
		vector<shared_ptr<Evaluator>> preferred_list;
		opts.set("preferred", preferred_list);
		engine = make_shared<AStarSolveAll>(opts);
	}

	return engine;
}

static options::Plugin<SearchEngine> _plugin("compute_hstar", _parse);
}
