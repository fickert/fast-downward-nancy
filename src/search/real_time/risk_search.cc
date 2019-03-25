
namespace real_time
{
  std::unique_ptr<StateOpenList> RiskLookaheadSearch::create_open_list()
  {
    auto options = options::Options();
    options.set("evals", std::vector<std::shared_ptr<Evaluator>>{f_hat_evaluator, heuristic});
    options.set("pref_only", false);
    options.set("unsafe_pruning", false);
    return std::make_unique<tiebreaking_open_list::TieBreakingOpenListFactory>(options)
      ->create_state_open_list();
  }


  // This generates the belief distribution from a search node.  Since
  // we only really care about the belief distribution of the TLAs.
  // Nancy just needs takes the belief of one particular node as the
  // belief for some TLA, so I'm pretty sure we don't need to store
  // the belief for each node, and instead only store the belief of
  // the TLA.
  DiscreteDistribution node_belief(SearchNode node)
  {
    auto eval_context = EvaluationContext(node.get_state(), node.get_g(), false, nullptr, false);
    const auto f = eval_context.get_evaluator_value_or_infinity(f_evaluator.get());
    const auto f_hat = eval_context.get_evaluator_value_or_infinity(f_hat_evaluator.get());
    const auto d = eval_context.get_evaluator_value_or_infinity(distance_heuristic.get());
    return DiscreteDistribution(100, f, f_hat, d, f_hat - f);
  }

  void TLAs::clear()
  {
    ops.clear();
    open_lists.clear();
    beliefs.clear();
    expected_min_costs.clear();
  }

  void TLAs::generate(successor_generator::SuccessorGenerator const &successor_generator,
                      GlobalState const &state, StateOpenList *open_list)
  {
    successor_generator.generate_applicable_ops(current_state, ops);
    // TODO: possibly rearrange this loop into several loops
    for (OperatorID op_id : ops) {
      auto const op = task_proxy.get_operators()[op_id];
      auto const succ_state = state_registry.get_successor_state(current_state, op);
      auto succ_node = search_space->get_node(succ_state);

      // add the node to the global open list and to this tla's open list
      open_lists.push_back(create_open_list());
      auto eval_context = EvaluationContext(succ_state, 0, false, statistics.get());
      open_list->insert(eval_context, succ_state.get_id());
      open_lists.back()->insert(eval_context, succ_state.get_id());
      if (expansion_delay) {
        open_list_insertion_time[succ_state.get_id()] = 0;
      }

      // add the belief (and expected value)
      beliefs.push_back(node_belief(succ_node));
      expected_min_costs.push_back(beliefs.back().expectedCost());
    }
  }

  void RiskLookaheadSearch::initialize(const GlobalState &initial_state)
  {
    LookaheadSearch::initialize(initial_state);

    this->current_state = &initial_state;
    this->open_list = create_open_list();

    // generate top level actions
    tlas.clear();
    tlas.generate(*current_state, open_list.get());
    // because we manually opened and expanded the initial state to generate the tlas
    statistics->inc_expanded_states();
    statistics->inc_generated(tlas.ops.size());

    closed.insert(search_space->get_node(initial_state));
  }

  double RiskLookaheadSearch::risk_analysis()
  {
    return 0;
  }

  // select the tla with the minimal expected risk
  int RiskLookaheadSearch::select_tla()
  {
    int res = 0;
    double min_risk = numeric_limits<double>::infinity();

    int alpha = 0;
    //double alpha_cost = tlas.beliefs[0].expectedCost();
    double alpha_cost = tlas.costs[0];

    for (int i = 1; i < tlas.costs.size(); ++i) {
      if (tlas.costs[i] < alpha_cost) {
        alpha_cost = tlas.costs[i];
        alpha = i;
      }
    }

    for (int i = 0; i < tlas.costs.size(); ++i) {
      if (tlas.open_lists[i].empty()) {
        continue;
      }
      // TODO: risk analysis
      double risk = 0.0;
      if (risk < min_risk) {
        min_risk = risk;
        res = i;
      }
    }
    return res;
  }

  SearchStatus RiskLookaheadSearch::search()
  {
    assert(statistics);
    state_owner.clear();
    while (statistics->getexpanded() < lookahead_bound) {
      if (open_list->empty())
        return FAILED;

      // TODO:
      // check if the beliefs of all tlas really need to be updated each time.
      // maybe it depends on the backup strategy, but I don't think it's necessary for nancy.
      int tla_id = select_tla();
      auto &open_list = tlas.open_lists[tla_id];
      auto state_id = open_list.remove_min();
      const auto owner = state_owner.find(state_id);
      if (owner == state_owner::end()) {
        // this should never happen
        // TODO: print error
        continue;
      }
      if (owner->second != tla_id) {
        // this might happen
        continue;
      }

      // Problem: here we need to remove this specific node from the global open list
      //
      auto state = state_registry.lookup_state(state_id);
      auto node = search_space->get_node(state);

      if (node.is_closed()) {
        continue;
      }

      mark_expanded(node);
      state_owner[node] = tla_id;

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
          predecessors[succ_state.get_id()].emplace_back(id, op);

        if (succ_node.is_dead_end())
          continue;

        if (succ_node.is_new()) {
          auto succ_eval_context = EvaluationContext(succ_state, node.get_g() + op.get_cost(), false, statistics.get());
          statistics->inc_evaluated_states();
          if (open_list->is_dead_end(succ_eval_context)) {
            succ_node.mark_as_dead_end();
            statistics->inc_dead_ends();
            continue;
          }
          succ_node.open(node, op, op.get_cost());
          open_list->insert(succ_eval_context, succ_state.get_id());
          state_owner[succ_state] = tla_id;
        } else if (succ_node.get_g() > node.get_g() + op.get_cost()) {
          // We found a new cheapest path to an open or closed state.
          if (succ_node.is_closed())
            statistics->inc_reopened();
          succ_node.reopen(node, op, op.get_cost());
          auto succ_eval_context = EvaluationContext(succ_state, succ_node.get_g(), false, statistics.get());
          open_list->insert(succ_eval_context, succ_state.get_id());
          state_owner[succ_state] = tla_id;
        }

        open_list_insertion_time[id] = statistics->get_expanded();
        if (heuristic_error)
          heuristic_error->add_successor(succ_node, op.get_cost());
      }
      if (heuristic_error)
        heuristic_error->update_error();
    }

    // collect the frontiers of all tlas
    for (int i = 0; i < open_lists.size(); ++i) {
      auto &open_list = open_lists[i];
      while (!open_list->empty()) {
        StateId state_id = open_list->remove_min();
        auto find = state_owner.find(state_id);
        if (find == state_owner::end()) {
          // this shouldn't happen
          // TODO: print error
          continue;
        }
        if (find->second != i) {
          continue;
        }
        frontier.push_back(state_id);
      }
    }
    return IN_PROGRESS;

    // (there's also something in the end to learn the one-step error. I'm not sure yet what that's about exactly).
    }
  }

  RiskLookaheadSearch::RiskLookaheadSearch(StateRegistry &state_registry, int lookahead_bound,
      std::shared_ptr<Evaluator> heuristic, std::shared_ptr<Evaluator> distance,
      bool store_exploration_data, ExpansionDelay *expansion_delay, HeuristicError &heuristic_error)
    : LookaheadSearch(state_registry, lookahead_bound, store_exploration_data,
                      expansion_delay, heuristic_error),
      f_hat_evaluator(create_f_hat_evaluator(heuristic, distance, heuristic_error)),
      heuristic(heuristic)
  {
    tlas.reserve(32);
    applicables.reserve(32);
  }


}
