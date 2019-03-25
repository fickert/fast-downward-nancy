#include "risk_search.h"

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


namespace real_time
{
  std::unique_ptr<StateOpenList> RiskLookaheadSearch::create_open_list() const
  {
    auto options = options::Options();
    options.set("evals", std::vector<std::shared_ptr<Evaluator>>{f_hat_evaluator, heuristic});
    options.set("pref_only", false);
    options.set("unsafe_pruning", false);
    return std::make_unique<tiebreaking_open_list::TieBreakingOpenListFactory>(options)
      ->create_state_open_list();
  }

  bool state_owned_by_tla(std::unordered_map<StateID, int> const &state_owner, StateID state_id, int tla_id)
  {
    const auto owner = state_owner.find(state_id);
    if (owner == state_owner.end()) {
      // this should never happen
      // TODO: print error
      return false;
    }
    if (owner->second != tla_id) {
      // this might happen
      return false;
    }
    return true;
  }

  // This generates the belief distribution from a search node.  Since
  // we only really care about the belief distribution of the TLAs.
  // Nancy just needs takes the belief of one particular node as the
  // belief for some TLA, so I'm pretty sure we don't need to store
  // the belief for each node, and instead only store the belief of
  // the TLA.
  DiscreteDistribution RiskLookaheadSearch::node_belief(SearchNode const &node)
  {
    auto eval_context = EvaluationContext(node.get_state(), node.get_g(), false, nullptr, false);
    const auto f = eval_context.get_evaluator_value_or_infinity(heuristic.get());
    const auto f_hat = eval_context.get_evaluator_value_or_infinity(f_hat_evaluator.get());
    const auto d = eval_context.get_evaluator_value_or_infinity(distance_heuristic.get());
    return DiscreteDistribution(100, f, f_hat, d, f_hat - f);
  }

  size_t TLAs::size() const
  {
    assert(ops.size() == open_lists.size() &&
           ops.size() == beliefs.size() &&
           ops.size() == expected_min_costs.size() &&
           ops.size() == states.size());
    return ops.size();
  }

  void TLAs::clear()
  {
    ops.clear();
    open_lists.clear();
    beliefs.clear();
    expected_min_costs.clear();
    states.clear();
  }

  void TLAs::reserve(uint n)
  {
    ops.reserve(n);
    open_lists.reserve(n);
    beliefs.reserve(n);
    expected_min_costs.reserve(n);
    states.reserve(n);
  }

  void RiskLookaheadSearch::generate_tlas(GlobalState const &current_state)
  {
    tlas.clear();
    successor_generator.generate_applicable_ops(current_state, tlas.ops);

    for (uint i = 0; i < tlas.ops.size(); ++i) {
      auto op_id = tlas.ops[i];
      auto const op = task_proxy.get_operators()[op_id];
      auto const succ_state = state_registry.get_successor_state(current_state, op);
      auto succ_node = search_space->get_node(succ_state);

      // add the node to this tla's open list
      tlas.open_lists.push_back(create_open_list());
      auto eval_context = EvaluationContext(succ_state, 0, false, statistics.get());
      tlas.open_lists.back()->insert(eval_context, succ_state.get_id());
      if (expansion_delay) {
        open_list_insertion_time[succ_state.get_id()] = 0;
      }
      state_owner[succ_state.get_id()] = i;

      // add the belief (and expected value)
      tlas.beliefs.push_back(node_belief(succ_node));
      tlas.expected_min_costs.push_back(tlas.beliefs.back().expectedCost());

      // add the node of the tla
      tlas.states.push_back(succ_state.get_id());
    }
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

  double RiskLookaheadSearch::risk_analysis(uint const alpha) const
  {
    double risk = 0.0;

    // integrate over probability nodes in alpha's belief
    // iterate over all other tlas beta
    // integrate over probability nodes in beta's belief
    // add to risk if beta cost is smaller than alpha cost
    // => risk is proportional to the chance that alpha isn't the optimal choice
    for (auto const &a : tlas.beliefs[alpha]) {
      for (uint beta = 0; beta < tlas.size(); ++beta) {
        if (alpha == beta)
          continue;
        for (auto const &b : tlas.beliefs[beta]) {
          if (b.cost < a.cost)
            risk += a.probability * b.probability * (a.cost - b.cost);
          else
            break;
        }
      }
    }
    return risk;
  }

  // select the tla with the minimal expected risk
  uint RiskLookaheadSearch::select_tla() const
  {
    // TODO: not sure if this is smart, setting up all these eval
    // contexts just to do the risk tie breaking, especially since we
    // don't reuse them.  maybe there's a smarter way.
    std::vector<EvaluationContext> eval_contexts;
    eval_contexts.reserve(tlas.size());
    for (uint i = 0; i < tlas.size(); ++i) {
      auto state = state_registry.lookup_state(tlas.states[i]);
      eval_contexts.emplace_back(state,
                                 search_space->get_node(state).get_g(),
                                 false,
                                 statistics.get());
    }

    uint res = 0;
    double min_risk = numeric_limits<double>::infinity();

    uint alpha = 0;
    //double alpha_cost = tlas.beliefs[0].expectedCost();
    double alpha_cost = tlas.expected_min_costs[0];

    for (uint i = 1; i < tlas.size(); ++i) {
      if (tlas.expected_min_costs[i] < alpha_cost) {
        alpha_cost = tlas.expected_min_costs[i];
        alpha = i;
      }
    }

    for (uint i = 0; i < tlas.size(); ++i) {
      if (tlas.open_lists[i]->empty()) {
        continue;
      }
      // TODO: there is something missing here.  andrew does some
      // squishing before the risk analysis.  Not quite sure how that
      // works.  In particular it requires knowledge about the
      // "average delay window"?
      double risk = risk_analysis(alpha);

      // keep the minimum risk tla.
      // otherwise tie-break f_hat -> f -> g
      if (risk < min_risk) {
        min_risk = risk;
        res = i;
      } else if (risk == min_risk) {
        auto &eval_context = eval_contexts[i];
        auto &min_eval_context = eval_contexts[res];
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

  void RiskLookaheadSearch::backup_beliefs()
  {
    // Originally, this function sets the k best nodes for each tla.
    // These k best nodes are then used during the decision phase.
    // Nothing else changes during the backup (!).  We don't use
    // kBestNodes in the decision phase, we don't even store them per
    // TLA.  So, backup_beliefs doesn't really do anything.

    // for (uint i = 0; i < tlas.size(); ++i) {
    //   auto open_list = tlas.open_lists[i];
    //   while (1) {
    //     if (open_list.empty())
    //       break;

    //     auto state_id = open_list->remove_min();
    //     if (state_owned_by_tla(state_owner, state_id, tla_id)) {
    //       DiscreteDistribution best = node_belief(search_space->get_node(state_registry.lookup_state(state_id)));
    //     }
    //   }
    // }
  }

  SearchStatus RiskLookaheadSearch::search()
  {
    assert(statistics);
    state_owner.clear();
    while (statistics->get_expanded() < lookahead_bound) {

      // setup work: find tla to expand under
      backup_beliefs();
      int tla_id = select_tla();
      if (tlas.open_lists[tla_id]->empty())
          return FAILED;

      // get the state to expand, discarding states that are not owned
      StateID state_id = tlas.open_lists[tla_id]->remove_min();
      while (1) {
        if (state_owned_by_tla(state_owner, state_id, tla_id))
          break;
        if (tlas.open_lists[tla_id]->empty())
          return FAILED;
        state_id = tlas.open_lists[tla_id]->remove_min();
      }

      auto state = state_registry.lookup_state(state_id);
      auto node = search_space->get_node(state);

      if (node.is_closed()) {
        continue;
      }

      mark_expanded(node);
      state_owner[state_id] = tla_id;

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
          auto succ_eval_context = EvaluationContext(succ_state, node.get_g() + op.get_cost(), false, statistics.get());
          statistics->inc_evaluated_states();
          if (tlas.open_lists[tla_id]->is_dead_end(succ_eval_context)) {
            succ_node.mark_as_dead_end();
            statistics->inc_dead_ends();
            continue;
          }
          succ_node.open(node, op, op.get_cost());
          tlas.open_lists[tla_id]->insert(succ_eval_context, succ_state.get_id());
          state_owner[succ_state.get_id()] = tla_id;
        } else if (succ_node.get_g() > node.get_g() + op.get_cost()) {
          // We found a new cheapest path to an open or closed state.
          if (succ_node.is_closed())
            statistics->inc_reopened();
          succ_node.reopen(node, op, op.get_cost());
          auto succ_eval_context = EvaluationContext(succ_state, succ_node.get_g(), false, statistics.get());
          tlas.open_lists[tla_id]->insert(succ_eval_context, succ_state.get_id());
          state_owner[succ_state.get_id()] = tla_id;
        }

        open_list_insertion_time[state_id] = statistics->get_expanded();
        if (heuristic_error)
          heuristic_error->add_successor(succ_node, op.get_cost());
      }
      if (heuristic_error)
        heuristic_error->update_error();
    }

    // collect the frontiers of all tlas
    for (uint i = 0; i < tlas.open_lists.size(); ++i) {
      while (!tlas.open_lists[i]->empty()) {
        StateID state_id = tlas.open_lists[i]->remove_min();
        if (state_owned_by_tla(state_owner, state_id, i))
          frontier.push_back(state_id);
      }
    }
    return IN_PROGRESS;

    // (there's also something in the end to learn the one-step error. I'm not sure yet what that's about exactly).
  }

  RiskLookaheadSearch::RiskLookaheadSearch(StateRegistry &state_registry, int lookahead_bound,
      std::shared_ptr<Evaluator> heuristic, std::shared_ptr<Evaluator> distance,
      bool store_exploration_data, ExpansionDelay *expansion_delay, HeuristicError *heuristic_error)
    : LookaheadSearch(state_registry, lookahead_bound, store_exploration_data,
                      expansion_delay, heuristic_error),
      f_hat_evaluator(create_f_hat_evaluator(heuristic, distance, *heuristic_error)),
      heuristic(heuristic),
      distance_heuristic(distance)
  {
    tlas.reserve(32);
    applicables.reserve(32);
    state_owner.reserve(lookahead_bound);
  }
}
