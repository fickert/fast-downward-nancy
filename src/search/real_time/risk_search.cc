
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

      // add the operator
      ops.push_back(op_id);

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

  SearchStatus RiskLookaheadSearch::search()
  {
    assert(statistics);
    while (statistics->getexpanded() < lookahead_bound) {
      if (open_list->empty())
        return FAILED;

      // TODO:
      // check if the beliefs of all tlas really need to be updated each time.
      // maybe it depends on the backup strategy, but I don't think it's necessary for nancy.
      int tla_id = select_tla();
      auto state_id = tlas.open_lists[tla_id].remove_min();
      // Problem: here we need to remove this specific node from the global open list
      auto state = state_registry.lookup_state(state_id);
      auto node = search_space->get_node(state);

      mark_expanded(node);
      // TODO: also remember which tla expanded this node

      if (check_goal_and_set_plan(state))
        return SOLVED;

      // TODO:
      // 1. simulate exapnsion under each tla to select minimum risk tla.
      // 2. select node from the open list of that tla.
      // 3. check for goal.
      // 4. remove node from global open, and tla open, (also increment expansio statistics).
      // 5. generate successor nodes, do duplicate detection.
      //    if this node is open, but another tla closed a node for that state before, keep better g value.
      //    Problem: We have to remove this specific node from the open list of the other tla.
      //    if the node is closed, proceed as usual, compare the f values and reopen it.
      //    if the node is new, just evaluate it and add it to open.


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
  }


}
