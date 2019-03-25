#ifndef RISK_SEARCH_H
#define RISK_SEARCH_H

#include "../abstract_task.h"
#include "../evaluator.h"
#include "../open_list.h"
#include "../plan_manager.h"
#include "../search_engine.h"
#include "../search_space.h"
#include "../state_registry.h"
#include "../task_utils/successor_generator.h"

#include <memory>
#include <vector>
#include <unordered_map>

namespace real_time
{
  struct TLAs
  {
    using Cost = double;
    std::vector<OperatorID> ops;
    std::vector<std::unique_ptr<StateOpenList>> open_lists;
    std::vector<DiscreteDistribution> beliefs;
    std::vector<Cost> expected_min_costs;
    void clear();
    void generate(successor_generator::SuccessorGenerator const &successor_generator,
                  GlobalState const &state, StateOpenList *open_list);
  };

  class RiskLookaheadSearch : public LookaheadSearch
  {
    // The current_state is set during initialize each step.  It's
    // stored in the RealTimeSearch class and stays valid until after
    // the search.  Therefore, storing it as a pointer here is fine.
    GlobalState *current_state;
    std::unique_ptr<StateOpenList> open_list;
    std::shared_ptr<Evaluator> f_hat_evaluator;
    std::shared_ptr<Evaluator> heuristic;
    TLAs tlas;
    
    // stores the index of the tla that owns the state
    // this is a hack to detect and prevent a state being expanded
    // under a tla when there is a different tla that has a shorter
    // path to that state.
    std::unordered_map<StateID, int> state_owner;

    // This is storage for applicable operators
    // It's kept here in the class because clearing a vector is
    // more efficient than creating a new one each iteration
    std::vector<OperatorID> applicables;
  public:
    RiskLookaheadSearch(StateRegistry &state_registry,
                        int lookahead_bound,
                        std::shared_ptr<Evaluator> heuristic,
                        std::shared_ptr<Evaluator> distance,
                        bool store_exploration_data,
                        ExpansionDelay *expansion_delay,
                        HeuristicError &heuristic_error);
    ~RiskLookaheadSearch() override = default;

    void initialize(const GlobalState &initial_state) override;
    auto search() -> SearchStatus override;

    RiskLookaheadSearch(const RiskLookaheadSearch &) = delete;
    RiskLookaheadSearch(RiskLookaheadSearch &&) = delete;

    auto operator=(const RiskLookaheadSearch &) = delete;
    auto operator=(RiskLookaheadSearch &&) = delete;
  };

}


#endif
