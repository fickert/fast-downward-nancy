#include "risk_search.h"

#include "expansion_delay.h"
#include "heuristic_error.h"
#include "util.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"
#include "../open_lists/tiebreaking_open_list.h"
#include "../options/plugin.h"
#include "../task_utils/task_properties.h"

namespace real_time
{

bool RiskLookaheadSearch::state_owned_by_tla(StateID state_id, int tla_id) const
{
	const auto &owner = state_owners.find(state_id);
	if (owner == state_owners.end()) {
		// this should never happen
		std::cerr << "Encountered state with no known owner\n";
		assert(false);
		return false;
	}
	return owner->second == tla_id;
}

void RiskLookaheadSearch::make_state_owner(StateID state_id, int tla_id)
{
	state_owners[state_id] = tla_id;
}

ShiftedDistribution RiskLookaheadSearch::get_belief(EvaluationContext &eval_context, int ph)
{
	// first check if we know this state from previous expansions and
	// have a belief about it
	auto const &state = eval_context.get_state();
	ShiftedDistribution belief = beliefs[state];
	if (nullptr != belief.distribution) {
		return belief;
	}

	// if not, get the distribution associated with the state's features
	// and assign the state's belief to that.
	int h = eval_context.get_evaluator_value(base_heuristic.get());
	DataFeature df(f_kind, h, ph);
	DiscreteDistribution *distribution = raw_beliefs.get_distribution(df);
	if (!distribution) {
		// this should never happen if we're using the data-driven approach
		const auto f = eval_context.get_evaluator_value_or_infinity(f_evaluator.get());
		assert(f != EvaluationResult::INFTY);
		const auto f_hat = eval_context.get_evaluator_value_or_infinity(f_hat_evaluator.get());
		assert(f_hat != EvaluationResult::INFTY);
		const auto d = eval_context.get_evaluator_value_or_infinity(distance_heuristic.get());
		assert(d != EvaluationResult::INFTY);
		distribution = new DiscreteDistribution(MAX_SAMPLES, f, f_hat, d, f_hat - f);
		raw_beliefs.remember(df, distribution);
	}
	belief.set(distribution, 0);
	beliefs[state] = belief;

	// It's more convenient to look up the post expansions belief here
	// already.  This way we can be sure that both beliefs are always
	// present.
	ShiftedDistribution post_belief;
	DataFeature post_df(pf_kind, h, ph);
	DiscreteDistribution *post_distribution = raw_post_beliefs.get_distribution(post_df);
	if (!post_distribution) {
		// this should never happen if we're using the data-driven approach
		++post_expansion_belief_gaussian_fallback_count;
		double ds = 1 / expansion_delay->get_avg_expansion_delay();
		auto eval_context2 = EvaluationContext(state_registry.lookup_state(state.get_id()), -1, false, nullptr);
		assert(!eval_context2.is_evaluator_value_infinite(distance_heuristic.get()));
		double dy = eval_context2.get_evaluator_value(distance_heuristic.get());
		double squishFactor = std::min(1.0, (ds / dy));
		// needs to be a copy, because these are separate distributions
		post_distribution = new DiscreteDistribution(distribution);
		post_distribution->squish(squishFactor);
		raw_post_beliefs.remember(post_df, post_distribution);
	}
	post_belief.set(post_distribution, 0);
	post_beliefs[state] = post_belief;

	assert(belief.distribution);
	assert(post_belief.distribution);

	return belief;
}

// Post expansion belief is generated when a node is created.  It is
// therefore guaranteed to be cached already.
ShiftedDistribution RiskLookaheadSearch::get_post_belief(StateID state_id)
{
	auto const &state = state_registry.lookup_state(state_id);
	ShiftedDistribution post_belief = post_beliefs[state];
	assert(post_belief.distribution);
	return post_belief;
}

void RiskLookaheadSearch::initialize(const GlobalState &initial_state)
{
	LookaheadSearch::initialize(initial_state);

	// generate tlas
	tlas.clear();
	state_owners.clear();
	if (heuristic_error)
		heuristic_error->set_expanding_state(initial_state);
	applicables.clear();
	successor_generator.generate_applicable_ops(initial_state, applicables);
	auto root_node = search_space->get_node(initial_state);
	auto const cur_state_id = initial_state.get_id();
	tlas.current_state = &initial_state;
	EvaluationContext root_eval_context = EvaluationContext(initial_state, 0, false, statistics.get());
	int root_h = root_eval_context.get_evaluator_value_or_infinity(heuristic.get());

	for (auto op_id : applicables) {
		auto const op = task_proxy.get_operators()[op_id];
		auto const succ_state = state_registry.get_successor_state(initial_state, op);
		auto const succ_state_id = succ_state.get_id();
		auto succ_node = search_space->get_node(succ_state);
		auto const adj_cost = search_engine->get_adjusted_cost(op);
		auto const succ_g = adj_cost;
		if (succ_node.is_new()) {
			succ_node.open(root_node, op, succ_g);
		} else if (!succ_node.is_dead_end() && succ_g < succ_node.get_g()) {
			succ_node.reopen(root_node, op, succ_g);
		} else {
			continue;
		}

		if (store_exploration_data)
			predecessors[succ_state_id].emplace_back(cur_state_id, op);

		auto eval_context = EvaluationContext(succ_state, succ_g, false, statistics.get());
		if (eval_context.is_evaluator_value_infinite(heuristic.get()) || eval_context.is_evaluator_value_infinite(distance_heuristic.get())) {
			succ_node.mark_as_dead_end();
			statistics->inc_dead_ends();
			continue;
		}

		tlas.ops.push_back(op_id);
		tlas.op_costs.push_back(adj_cost);

		// add the belief
		tlas.beliefs.push_back(get_belief(eval_context, root_h));
		tlas.post_beliefs.push_back(get_post_belief(succ_state_id));

		// add the node to this tla's open list
		tlas.open_lists.emplace_back();
		tlas.open_lists.back().emplace(NodeEvaluation(tlas.beliefs.back().expected_cost(),
							      succ_g,
							      eval_context.get_result(heuristic.get()).get_evaluator_value()),
					       succ_state_id);

		// add the tla state
		tlas.states.emplace_back(succ_state_id, tlas.beliefs.back().expected_cost());

		// add the context for the tla's state
		tlas.eval_contexts.emplace_back(std::move(eval_context));

		if (expansion_delay) {
			open_list_insertion_time[succ_state_id] = 0;
		}
		// make the tla own the state of the top level node (I assume
		// here, that the state of all top level nodes are distinct)
		assert(state_owners[succ_state_id].empty());
		make_state_owner(succ_state_id, static_cast<int>(tlas.ops.size()) - 1);

		if (heuristic_error)
			heuristic_error->add_successor(succ_node, adj_cost);
	}

	mark_expanded(root_node);
	statistics->inc_generated(tlas.size());

	if (heuristic_error)
		heuristic_error->update_error();


#ifndef DNDEBUG
	for (size_t i = 0; i < tlas.size(); ++i) {
		assert(tlas.open_lists[i].size() == 1);

		if (!(tlas.post_beliefs[i].expected_cost() -
		      (get_post_belief(tlas.min(i).second).expected_cost() + tlas.min(i).first.g - tlas.op_costs[i])
		      < 0.01)) {
			std::cout << i << ", "
				  << (void*)tlas.post_beliefs[i].distribution << ", "
				  << tlas.post_beliefs[i].expected_cost() << ", "
				  << (void*)get_post_belief(tlas.min(i).second).distribution << ", "
				  << get_post_belief(tlas.min(i).second).expected_cost() << ", "
				  << tlas.min(i).first.g << ", "
				  << tlas.op_costs[i] << "\n";
			assert(false);
		}
	}
#endif
}

double RiskLookaheadSearch::risk_analysis(size_t const alpha, const std::vector<ShiftedDistribution> &est_beliefs) const
{
	double risk = 0.0;

	// integrate over probability nodes in alpha's belief
	// iterate over all other tlas beta
	// integrate over probability nodes in beta's belief
	// add to risk if beta cost is smaller than alpha cost
	// => risk is proportional to the chance that alpha isn't the optimal choice
	for (auto const &a : *est_beliefs[alpha].distribution) {
		double shifted_a_cost = a.cost + est_beliefs[alpha].shift;
		for (size_t beta = 0; beta < tlas.size(); ++beta) {
			assert(est_beliefs[beta].distribution != nullptr);
			if (alpha == beta)
				continue;
			for (auto const &b : *est_beliefs[beta].distribution) {
				double shifted_b_cost = b.cost + est_beliefs[beta].shift;
				if (shifted_b_cost < shifted_a_cost)
					risk += a.probability * b.probability * (shifted_a_cost - shifted_b_cost);
				else
					break;
			}
		}
	}
	return risk;
}

// select the tla with the minimal expected risk
size_t RiskLookaheadSearch::select_tla()
{
	size_t res = 0;
	double min_risk = std::numeric_limits<double>::infinity();

	size_t alpha = 0;
	double alpha_cost = tlas.beliefs[0].expected_cost();

	for (size_t i = 1; i < tlas.size(); ++i) {
		if (tlas.beliefs[i].expected_cost() < alpha_cost) {
			alpha_cost = tlas.beliefs[i].expected_cost();
			alpha = i;
		}
	}

	for (size_t i = 0; i < tlas.size(); ++i) {
		if (tlas.open_lists[i].empty()) {
			continue;
		}
		assert(expansion_delay);
		assert(tlas.post_beliefs[i].expected_cost() -
		       (get_post_belief(tlas.open_lists[i].top().second).expected_cost() + tlas.open_lists[i].top().first.g - tlas.op_costs[i])
		       < 0.01);

		// Simulate how expanding this TLA's best node would affect its belief
		ShiftedDistribution post_i = tlas.post_beliefs[i];
		assert(post_i.distribution);
		// swap in the estimated post expansion belief
		ShiftedDistribution tmp = tlas.beliefs[i];
		assert(tmp.distribution);
		tlas.beliefs[i] = post_i;
		double risk = risk_analysis(alpha, tlas.beliefs);
		// restore the previous one
		tlas.beliefs[i] = tmp;

		// keep the minimum risk tla.
		// otherwise tie-break f_hat -> f -> g
		if (risk < min_risk) {
			min_risk = risk;
			res = i;
		} else if (double_eq(risk, min_risk)) {
			double min_f_hat = tlas.open_lists[res].empty()
				? std::numeric_limits<double>::infinity()
				: tlas.min(res).first.expected;
			double f_hat = tlas.open_lists[i].empty()
				? std::numeric_limits<double>::infinity()
				: tlas.min(i).first.expected;
			if (f_hat < min_f_hat) {
				min_risk = risk;
				res = i;
			} else if (double_eq(f_hat, min_f_hat)) {
				auto &eval_context = tlas.eval_contexts[i];
				auto &min_eval_context = tlas.eval_contexts[res];
				int min_f = min_eval_context.get_evaluator_value_or_infinity(heuristic.get())
					+ min_eval_context.get_g_value();
				int f = eval_context.get_evaluator_value_or_infinity(heuristic.get())
					+ eval_context.get_g_value();
				if (f < min_f) {
					min_risk = risk;
					res = i;
				} else if (f == min_f) {
					int min_g = min_eval_context.get_g_value();
					int g = eval_context.get_g_value();
					if (g < min_g) {
						min_risk = risk;
						res = i;
					}
				}
			}
		}
	}

	if (double_eq(alpha_cost, tlas.beliefs[res].expected_cost()))
		++alpha_expansion_count;
	else
		++beta_expansion_count;

	return res;
}

// nancy just backs up the top of the open list
void RiskLookaheadSearch::backup_beliefs()
{
	// std::cout << "backing up beliefs\n";
	// note that we need to update all beliefs, because one tla can
	// potentially "steal" the currently best belief of another tla
	for (size_t tla_id = 0; tla_id < tlas.size(); ++tla_id) {
		// this while loop just loops until it finds a state in the open
		// list that is owned by the tla
		while (1) {
			if (tlas.open_lists[tla_id].empty()) {
				tlas.beliefs[tla_id] = dead_end_belief;
				break;
			}

			// get the best state from open
			auto const &best_state = tlas.min(tla_id);
			auto const state_id = best_state.second;
			// we want the g from best_state to top level node, not to root
			auto const g = best_state.first.g - tlas.op_costs[tla_id];
			// only consider this state if we own it
			if (state_owned_by_tla(state_id, tla_id)) {
				// only do backup if it's a different state from last time
				if (state_id != tlas.states[tla_id].first) {
					auto best_state = state_registry.lookup_state(state_id);
					// every known node should have a known belief
					assert(beliefs[best_state].distribution != nullptr);
					assert(post_beliefs[best_state].distribution != nullptr);

					tlas.states[tla_id] = std::make_pair(state_id, beliefs[best_state].expected_cost());

					tlas.beliefs[tla_id].set_and_shift(beliefs[best_state], g);
					tlas.post_beliefs[tla_id].set_and_shift(post_beliefs[best_state], g);
					tlas.eval_contexts[tla_id] = EvaluationContext(best_state, g, false, statistics.get());
					assert(beliefs[best_state].expected_cost() + g - tlas.beliefs[tla_id].expected_cost() < 0.01);
				}
				// we're done in any case, since we found the best owned node
				// for this tla.
				break;
			} else {
				tlas.remove_min(tla_id);
			}
		}
	}

}

SearchStatus RiskLookaheadSearch::step()
{
	assert(statistics);

	if (tlas.size() == 0u) {
		return FAILED;
	}

	// setup work: find tla to expand under
	backup_beliefs();

get_node:
	int tla_id = select_tla();
	if (tlas.open_lists[tla_id].empty()) {
		// Note: this is safe, we'll never try to expand an empty tla,
		// if there is a non-empty one.
		return FAILED;
	}

	// get the state to expand, discarding states that are not owned
	auto const &top = tlas.remove_min(tla_id);
	StateID state_id = top.second;
	int this_h = top.first.h;
	while (1) {
		if (state_owned_by_tla(state_id, tla_id))
			break;
		if (tlas.open_lists[tla_id].empty()) {
			return FAILED;
		}
		state_id = tlas.remove_min(tla_id).second;
	}

	auto state = state_registry.lookup_state(state_id);
	auto node = search_space->get_node(state);

	if (node.is_closed())
		goto get_node;

	mark_expanded(node);
	assert(state_owned_by_tla(state_id, tla_id));

	if (check_goal_and_set_plan(state)) {
		return SOLVED;
	}

	applicables.clear();
	successor_generator.generate_applicable_ops(state, applicables);
	auto eval_context = EvaluationContext(state, node.get_g(), false, statistics.get());
	if (heuristic_error)
		heuristic_error->set_expanding_state(state);

	for (auto op_id : applicables) {
		const auto op = task_proxy.get_operators()[op_id];
		const auto adj_cost = search_engine->get_adjusted_cost(op);
		const auto succ_state = state_registry.get_successor_state(state, op);
		const auto succ_state_id = succ_state.get_id();
		statistics->inc_generated();
		auto succ_node = search_space->get_node(succ_state);
		auto succ_g = node.get_g() + adj_cost;

		if (store_exploration_data)
			predecessors[succ_state_id].emplace_back(state_id, op);

		if (succ_node.is_dead_end())
			continue;

		auto succ_eval_context = EvaluationContext(succ_state, succ_g, false, statistics.get());

		if (succ_node.is_new()) {
			statistics->inc_evaluated_states();
			bool is_dead_end =
				succ_eval_context.is_evaluator_value_infinite(f_hat_evaluator.get()) &&
				succ_eval_context.is_evaluator_value_infinite(heuristic.get());
			if (is_dead_end) {
				succ_node.mark_as_dead_end();
				statistics->inc_dead_ends();
				continue;
			}
			succ_node.open(node, op, adj_cost);
			auto belief = get_belief(succ_eval_context, this_h);
			tlas.open_lists[tla_id].emplace(NodeEvaluation(belief.expected_cost(),
								       succ_g,
								       succ_eval_context.get_result(heuristic.get()).get_evaluator_value()),
							succ_state_id);
			make_state_owner(succ_state_id, tla_id);
		} else {
			auto const old_g = succ_node.get_g();
			auto const new_g = node.get_g() + adj_cost;
			if (old_g > new_g) {
				// new cheapest path to this state
				if (succ_node.is_closed())
					statistics->inc_reopened();
				succ_node.reopen(node, op, adj_cost);
				auto belief = get_belief(succ_eval_context, this_h);
				tlas.open_lists[tla_id].emplace(NodeEvaluation(belief.expected_cost(),
									       new_g,
									       succ_eval_context.get_result(heuristic.get()).get_evaluator_value()),
								succ_state_id);
				make_state_owner(succ_state_id, tla_id);
			}
		}

		open_list_insertion_time[state_id] = statistics->get_expanded();
		if (heuristic_error)
			heuristic_error->add_successor(succ_node, adj_cost);
	}
	if (heuristic_error)
		heuristic_error->update_error();

	return IN_PROGRESS;
}

void RiskLookaheadSearch::post()
{
	backup_beliefs();
	for (size_t i = 0; i < tlas.open_lists.size(); ++i) {
		while (!tlas.open_lists[i].empty()) {
			StateID state_id = tlas.remove_min(i).second;
			if (state_owned_by_tla(state_id, i))
				frontier.push_back(state_id);
		}
	}
}

void RiskLookaheadSearch::print_statistics() const {
	std::cout << "Fallback to gaussian (node belief): " << hstar_gaussian_fallback_count << "\n"
		  << "Fallback to gaussian (post-expansion belief): " << post_expansion_belief_gaussian_fallback_count << "\n"
		  << "Number of expansions under alpha: " << alpha_expansion_count << "\n"
		  << "Number of expansions under beta: " << beta_expansion_count << "\n";
}

RiskLookaheadSearch::RiskLookaheadSearch(StateRegistry &state_registry,
					 std::shared_ptr<Evaluator> heuristic, std::shared_ptr<Evaluator> base_heuristic, std::shared_ptr<Evaluator> distance,
					 bool store_exploration_data, ExpansionDelay *expansion_delay, HeuristicError *heuristic_error,
					 HStarData<int> *hstar_data,
					 HStarData<long long> *post_expansion_belief_data,
					 SearchEngine const *search_engine,
					 DataFeatureKind f_kind,
					 DataFeatureKind pf_kind)
	: LookaheadSearch(state_registry, store_exploration_data,
			  expansion_delay, heuristic_error, search_engine),
	  f_evaluator(std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{heuristic, std::make_shared<g_evaluator::GEvaluator>()})),
	  f_hat_evaluator(create_f_hat_evaluator(heuristic, distance, *heuristic_error)),
	  heuristic(heuristic),
	  base_heuristic(base_heuristic),
	  distance_heuristic(distance),
	  beliefs(),
	  post_beliefs(),
	  raw_beliefs(f_kind, hstar_data),
	  raw_post_beliefs(pf_kind, post_expansion_belief_data),
	  f_kind(f_kind),
	  pf_kind(pf_kind),
	  hstar_data(hstar_data),
	  post_expansion_belief_data(post_expansion_belief_data),
	  hstar_gaussian_fallback_count(0),
	  post_expansion_belief_gaussian_fallback_count(0),
	  alpha_expansion_count(0),
	  beta_expansion_count(0)
{
	tlas.reserve(32);
	applicables.reserve(32);
	state_owners.reserve(256);
	assert(raw_beliefs.size() > 0 && raw_post_beliefs.size() > 0);

	dead_end_belief.set(&DiscreteDistribution::dead_distribution, 0);
}

RiskLookaheadSearch::~RiskLookaheadSearch()
{
	// for (auto b : raw_beliefs) {
	//   delete b;
	// }
	// for (auto b : raw_post_beliefs) {
	//   delete b;
	// }
	// delete dead_distribution;
}

}
