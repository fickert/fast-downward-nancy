#include "risk_search.h"

#include "expansion_delay.h"
#include "heuristic_error.h"
#include "util.h"
#include "../open_lists/tiebreaking_open_list.h"
#include "../options/plugin.h"
#include "../task_utils/task_properties.h"

#include <iostream>

namespace real_time
{

bool state_owned_by_tla(std::unordered_map<StateID, std::vector<int> > const &state_owners, StateID state_id, int tla_id)
{
  const auto &owners = state_owners.find(state_id);
  if (owners == state_owners.end()) {
    // this should never happen
    assert(false);
    return false;
  }
  auto &vec = owners->second;
  auto found = std::find(std::begin(vec), std::end(vec), tla_id);
  if (found == vec.end()) {
    return false;
  }
  return true;
}

void make_state_owner(std::unordered_map<StateID, std::vector<int> > &state_owners, StateID state_id, int tla_id)
{
  auto &owners = state_owners[state_id];
  owners.clear();
  owners.push_back(tla_id);
  assert(state_owners[state_id].size() == 1);
}

void add_state_owner(std::unordered_map<StateID, std::vector<int> > &state_owners, StateID state_id, int tla_id)
{
  auto owners = state_owners[state_id];
  auto found = std::find(std::begin(owners), std::end(owners), tla_id);
  if (found == owners.end()) {
    owners.push_back(tla_id);
  }
}

ShiftedDistribution RiskLookaheadSearch::node_belief(SearchNode const &node)
{
  // first check if we know this state from previous expansions and
  // have a belief about it
  auto const &state = node.get_state();
  ShiftedDistribution belief = beliefs[state];
  if (nullptr != belief.distribution) {
    return belief;
  }

  // if not, check if we know which distribution is associated with
  // the states h-value.  If yes, assign the state's belief to that.
  auto eval_context = EvaluationContext(node.get_state(), node.get_g(), false, nullptr, false);
  int h = eval_context.get_evaluator_value(base_heuristic.get());
  if (static_cast<size_t>(h) >= raw_beliefs.size()) {
    raw_beliefs.resize(h+1);
  }
  DiscreteDistribution *distribution = raw_beliefs[h];
  if (nullptr != distribution) {
    belief.set(distribution, 0);
    beliefs[state] = belief;
    return belief;
  }

  // if not, create the distribution for this h-value based on data,
  // or using gauss, cache it, and assign the state's belief to it.
  if (hstar_data) {
    const auto hstar_data_it = hstar_data->find(h);
    if (hstar_data_it != std::end(*hstar_data)) {
      distribution = new DiscreteDistribution(MAX_SAMPLES, hstar_data_it->second);
      raw_beliefs[h] = distribution;
      belief.set(distribution, 0);
      beliefs[state] = belief;
      return belief;
    }
    ++hstar_gaussian_fallback_count;
  }
  const auto f = eval_context.get_evaluator_value_or_infinity(f_evaluator.get());
  assert(f != EvaluationResult::INFTY);
  const auto f_hat = eval_context.get_evaluator_value_or_infinity(f_hat_evaluator.get());
  assert(f_hat != EvaluationResult::INFTY);
  const auto d = eval_context.get_evaluator_value_or_infinity(distance_heuristic.get());
  assert(d != EvaluationResult::INFTY);
  distribution = new DiscreteDistribution(MAX_SAMPLES, f, f_hat, d, f_hat - f);
  raw_beliefs[h] = distribution;
  belief.set(distribution, 0);
  beliefs[state] = belief;
  return belief;
}

// TODO: store post expansion beliefs
ShiftedDistribution RiskLookaheadSearch::post_expansion_belief(StateID best_state_id, ShiftedDistribution const *current_belief)
{
  auto eval_context = EvaluationContext(state_registry.lookup_state(best_state_id), -1, false, nullptr);
  int h = eval_context.get_evaluator_value(base_heuristic.get());

  if (static_cast<size_t>(h) >= raw_post_beliefs.size()) {
    raw_post_beliefs.resize(h+1);
  }

  DiscreteDistribution *distribution = raw_post_beliefs[h];
  ShiftedDistribution belief;
  if (distribution != nullptr) {
    belief.set(distribution, current_belief->shift);
    return belief;
  }

  if (post_expansion_belief_data) {
    const auto post_expansion_belief_data_it = post_expansion_belief_data->find(h);
    if (post_expansion_belief_data_it != std::end(*post_expansion_belief_data)) {
      distribution = new DiscreteDistribution(MAX_SAMPLES, post_expansion_belief_data_it->second);
      raw_post_beliefs[h] = distribution;
      belief.set(distribution, current_belief->shift);
      return belief;
    }
    ++post_expansion_belief_gaussian_fallback_count;
  }
  // Belief of TLA is squished as a result of search. Mean stays the same, but variance is decreased by a factor based on expansion delay.
  double ds = 1 / expansion_delay->get_avg_expansion_delay();
  auto best_state_eval_context = EvaluationContext(state_registry.lookup_state(best_state_id), -1, false, nullptr);
  assert(!best_state_eval_context.is_evaluator_value_infinite(distance_heuristic.get()));
  double dy = best_state_eval_context.get_evaluator_value(distance_heuristic.get());
  double squishFactor = min(1.0, (ds / dy));
  // make a copied instance of the current distribution, because the
  // original one must not be changed
  distribution = new DiscreteDistribution(current_belief->distribution);
  distribution->squish(squishFactor);
  belief.set(distribution, current_belief->shift);
  return belief;
}

size_t TLAs::size() const
{
  assert(ops.size() == open_lists.size() &&
         ops.size() == beliefs.size() &&
         ops.size() == states.size() &&
         ops.size() == eval_contexts.size());
  return ops.size();
}

void TLAs::clear()
{
  ops.clear();
  open_lists.clear();
  beliefs.clear();
  states.clear();
  eval_contexts.clear();
}

void TLAs::reserve(size_t n)
{
  ops.reserve(n);
  open_lists.reserve(n);
  beliefs.reserve(n);
  states.reserve(n);
  eval_contexts.reserve(n);
}

StateID TLAs::min(size_t tla_id) const
{
  return open_lists[tla_id].top().second;
}

StateID TLAs::remove_min(size_t tla_id)
{
  StateID res = min(tla_id);
  open_lists[tla_id].pop();
  return res;
}

void RiskLookaheadSearch::generate_tlas(GlobalState const &current_state)
{
  tlas.clear();
  state_owners.clear();
  if (heuristic_error)
    heuristic_error->set_expanding_state(current_state);
  auto ops = std::vector<OperatorID>();
  successor_generator.generate_applicable_ops(current_state, ops);
  auto root_node = search_space->get_node(current_state);

  for (auto op_id : ops) {
    auto const op = task_proxy.get_operators()[op_id];
    auto const succ_state = state_registry.get_successor_state(current_state, op);
    auto succ_node = search_space->get_node(succ_state);
    if (succ_node.is_new())
      succ_node.open(root_node, op, op.get_cost());
    else if (!succ_node.is_dead_end() && op.get_cost() < succ_node.get_g())
      succ_node.reopen(root_node, op, op.get_cost());
    else
      continue;

    auto eval_context = EvaluationContext(succ_state, succ_node.get_g(), false, statistics.get());
    if (eval_context.is_evaluator_value_infinite(heuristic.get()) || eval_context.is_evaluator_value_infinite(distance_heuristic.get())) {
      succ_node.mark_as_dead_end();
      statistics->inc_dead_ends();
      continue;
    }

    tlas.ops.push_back(op_id);

    // add the belief
    tlas.beliefs.push_back(node_belief(succ_node));

    // add the node to this tla's open list
    tlas.open_lists.emplace_back();
    tlas.open_lists.back().emplace(static_cast<double>(succ_node.get_g()) + tlas.beliefs.back().expected_cost(), succ_state.get_id());

    // add the context for the tla's state
    tlas.eval_contexts.emplace_back(std::move(eval_context));

    if (expansion_delay) {
      open_list_insertion_time[succ_state.get_id()] = 0;
    }
    // make the tla own the state of the top level node (I assume
    // here, that the state of all top level nodes are distinct)
    make_state_owner(state_owners, succ_state.get_id(), static_cast<int>(tlas.ops.size()) - 1);

    // add the state of the tla
    tlas.states.push_back(succ_state.get_id());

    if (heuristic_error)
      heuristic_error->add_successor(succ_node, op.get_cost());
  }
  root_node.close();
  if (heuristic_error)
    heuristic_error->update_error();
}

void RiskLookaheadSearch::initialize(const GlobalState &initial_state)
{
  LookaheadSearch::initialize(initial_state);

  // generate top level actions
  generate_tlas(initial_state);
  // because we manually opened and expanded the initial state to generate the tlas
  statistics->inc_expanded();
  statistics->inc_generated(tlas.size());

  if (store_exploration_data)
    closed.emplace(initial_state.get_id());
}

double RiskLookaheadSearch::risk_analysis(size_t const alpha, const vector<ShiftedDistribution> &beliefs) const
{
  double risk = 0.0;

  // integrate over probability nodes in alpha's belief
  // iterate over all other tlas beta
  // integrate over probability nodes in beta's belief
  // add to risk if beta cost is smaller than alpha cost
  // => risk is proportional to the chance that alpha isn't the optimal choice
  for (auto const &a : *beliefs[alpha].distribution) {
    double shifted_a_cost = a.cost + beliefs[alpha].shift;
    for (size_t beta = 0; beta < tlas.size(); ++beta) {
      if (alpha == beta)
        continue;
      for (auto const &b : *beliefs[beta].distribution) {
        double shifted_b_cost = b.cost + beliefs[beta].shift;
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
  double min_risk = numeric_limits<double>::infinity();

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
    // Simulate how expanding this TLA's best node would affect its belief
    ShiftedDistribution post_i = post_expansion_belief(tlas.open_lists[i].top().second, &tlas.beliefs[i]);
    // swap in the estimated post expansion belief
    ShiftedDistribution tmp = tlas.beliefs[i];
    tlas.beliefs[i] = post_i;
    double risk = risk_analysis(alpha, tlas.beliefs);
    // restore the previous one
    tlas.beliefs[i] = tmp;
    delete post_i.distribution;
    post_i.distribution = nullptr;

    // keep the minimum risk tla.
    // otherwise tie-break f_hat -> f -> g
    if (risk < min_risk) {
      min_risk = risk;
      res = i;
    } else if (risk == min_risk) {
      auto &eval_context = tlas.eval_contexts[i];
      auto &min_eval_context = tlas.eval_contexts[res];
      int min_f_hat = min_eval_context.get_evaluator_value_or_infinity(f_hat_evaluator.get());
      int f_hat = eval_context.get_evaluator_value_or_infinity(f_hat_evaluator.get());
      if (f_hat < min_f_hat) {
        min_risk = risk;
        res = i;
      } else if (f_hat == min_f_hat) {
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

  return res;
}

// nancy just backs up the top of the open list
void RiskLookaheadSearch::backup_beliefs()
{
  // note that we need to update all beliefs, because one tla can
  // potentially "steal" the currently best belief of another tla
  for (size_t tla_id = 0; tla_id < tlas.size(); ++tla_id) {
    // this while loop just loops until it finds a state in the open
    // list that is owned by the tla
    while (1) {
      if (tlas.open_lists[tla_id].empty()) {
        // TODO: it doesn't matter, but it's a little weird, we only
        // change the expected value here, but not the distribution.
        tlas.beliefs[tla_id].expected_value = numeric_limits<double>::infinity();
        break;
      }

      // get the best state from open
      auto state_id = tlas.min(tla_id);
      if (state_owned_by_tla(state_owners, state_id, tla_id)) {
        auto best_state = state_registry.lookup_state(state_id);
        auto best_node = search_space->get_node(best_state);
        // compute belief, or lookup if new
        tlas.beliefs[tla_id] = node_belief(best_node);
        assert(tlas.beliefs[tla_id].distribution != nullptr);
        break;
      } else {
        tlas.remove_min(tla_id);
      }
    }
  }
}

SearchStatus RiskLookaheadSearch::search()
{
  assert(statistics);
  while (statistics->get_expanded() < lookahead_bound) {

    if (tlas.size() == 0u) {
      return FAILED;
    }

    // setup work: find tla to expand under
    backup_beliefs();
    int tla_id = select_tla();
    if (tlas.open_lists[tla_id].empty()) {
      // Note: this is safe, we'll never try to expand an empty tla,
      // if there is a non-empty one.
      return FAILED;
    }


    // get the state to expand, discarding states that are not owned
    StateID state_id = tlas.remove_min(tla_id);
    while (1) {
      if (state_owned_by_tla(state_owners, state_id, tla_id))
        break;
      if (tlas.open_lists[tla_id].empty()) {
        std::cout << "no owned nodes in open lists under tla\n";
        return FAILED;
      }
      state_id = tlas.remove_min(tla_id);
    }

    auto state = state_registry.lookup_state(state_id);
    auto node = search_space->get_node(state);

    if (node.is_closed()) {
      continue;
    }

    mark_expanded(node);
    assert(state_owned_by_tla(state_owners, state_id, tla_id));

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
      const auto succ_state = state_registry.get_successor_state(state, op);
      statistics->inc_generated();
      auto succ_node = search_space->get_node(succ_state);

      if (store_exploration_data)
        predecessors[succ_state.get_id()].emplace_back(state_id, op);

      if (succ_node.is_dead_end())
        continue;

      if (succ_node.is_new()) {
        statistics->inc_evaluated_states();
        bool is_dead_end =
          eval_context.is_evaluator_value_infinite(f_hat_evaluator.get()) &&
          eval_context.is_evaluator_value_infinite(heuristic.get());
        if (is_dead_end) {
          succ_node.mark_as_dead_end();
          statistics->inc_dead_ends();
          continue;
        }
        succ_node.open(node, op, op.get_cost());
        auto belief = node_belief(succ_node);
        tlas.open_lists[tla_id].emplace(belief.expected_cost(), succ_state.get_id());
        make_state_owner(state_owners, succ_state.get_id(), tla_id);
      } else {
        auto const new_g = node.get_g() + op.get_cost();
        auto const old_g = succ_node.get_g();
        if (old_g >= new_g) {
          if (succ_node.is_closed())
            statistics->inc_reopened();
          succ_node.reopen(node, op, op.get_cost());
          auto belief = node_belief(succ_node);
          tlas.open_lists[tla_id].emplace(belief.expected_cost(), succ_state.get_id());

          if (old_g != new_g) {
            // new cheapest path to this state
            make_state_owner(state_owners, succ_state.get_id(), tla_id);
          } else {
            // there are cheapest paths below more than one tla leading to this state
            add_state_owner(state_owners, succ_state.get_id(), tla_id);
          }
        }
      }

      open_list_insertion_time[state_id] = statistics->get_expanded();
      if (heuristic_error)
        heuristic_error->add_successor(succ_node, op.get_cost());
    }
    if (heuristic_error)
      heuristic_error->update_error();
  }

  // collect the frontiers of all tlas
  for (size_t i = 0; i < tlas.open_lists.size(); ++i) {
    while (!tlas.open_lists[i].empty()) {
      StateID state_id = tlas.remove_min(i);
      if (state_owned_by_tla(state_owners, state_id, i))
        frontier.push_back(state_id);
    }
  }
  return IN_PROGRESS;

  // TODO: In Tianyi/Andrew's code there's also something here to
  // learn the one-step error. I'm not sure yet what that's about
  // exactly and how to translate it.
}

void RiskLookaheadSearch::print_statistics() const {
  std::cout << "Fallback to gaussian (node belief): " << hstar_gaussian_fallback_count << std::endl;
  std::cout << "Fallback to gaussian (post-expansion belief): " << post_expansion_belief_gaussian_fallback_count << std::endl;
}

RiskLookaheadSearch::RiskLookaheadSearch(StateRegistry &state_registry, int lookahead_bound,
                                         std::shared_ptr<Evaluator> heuristic, std::shared_ptr<Evaluator> base_heuristic, std::shared_ptr<Evaluator> distance,
                                         bool store_exploration_data, ExpansionDelay *expansion_delay, HeuristicError *heuristic_error, hstar_data_type<int> *hstar_data, hstar_data_type<long long> *post_expansion_belief_data)
  : LookaheadSearch(state_registry, lookahead_bound, store_exploration_data,
                    expansion_delay, heuristic_error),
    f_evaluator(std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{heuristic, std::make_shared<g_evaluator::GEvaluator>()})),
    f_hat_evaluator(create_f_hat_evaluator(heuristic, distance, *heuristic_error)),
    heuristic(heuristic),
    base_heuristic(base_heuristic),
    distance_heuristic(distance),
    beliefs(),
    hstar_data(hstar_data),
    post_expansion_belief_data(post_expansion_belief_data),
    hstar_gaussian_fallback_count(0),
    post_expansion_belief_gaussian_fallback_count(0)
{
  tlas.reserve(32);
  applicables.reserve(32);
  state_owners.reserve(lookahead_bound);
}

RiskLookaheadSearch::~RiskLookaheadSearch()
{
  for (auto b : raw_beliefs) {
    delete b;
  }
}


}
