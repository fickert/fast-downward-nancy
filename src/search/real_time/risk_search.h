#ifndef RISK_SEARCH_H
#define RISK_SEARCH_H

#include "lookhead_search.h"
#include "DiscreteDistribution.h"
#include "../evaluator.h"
#include "../open_list.h"
#include "../search_engine.h"
#include "../search_space.h"
#include "../state_registry.h"
#include "../per_state_information.h"

#include <memory>
#include <vector>
#include <unordered_map>

namespace real_time
{

// struct QueueEntryCompare
// {

// };

struct TLAs
{
  using QueueEntry = std::pair<double, StateID>;
  using QueueEntryCompare =
    struct {
      bool operator()(std::pair<double, StateID> const &l, std::pair<double, StateID> const &r) const
      {
       return l.first < r.first;
      }
    };
  using Queue =
    std::priority_queue<QueueEntry,
                        std::vector<QueueEntry>,
                        QueueEntryCompare>;
  std::vector<OperatorID> ops;
  std::vector<Queue> open_lists;
  // backed up belief for each tla
  std::vector<ShiftedDistribution> beliefs;
  std::vector<ShiftedDistribution> post_beliefs;
  std::vector<StateID> states;
  /*
  // "states" stores the top most states of each tla.
  // if a1,a2,a3 are the tlas, then "states" stores the state ids of the nodes n1,n2,n3:
  current_node
     /  |   \
   a1   a2   a3 ...
   /    |     \
  n1    n2     n3 ...
  */

  // TODO: not sure if this is smart, setting up all these eval
  // contexts
  std::vector<EvaluationContext> eval_contexts;


  void reserve(std::size_t n);
  void clear();
  size_t size() const;
  StateID remove_min(size_t tla_id);
  StateID min(size_t tla_id) const;
};

class RiskLookaheadSearch : public LookaheadSearch
{
  std::shared_ptr<Evaluator> f_evaluator;
  std::shared_ptr<Evaluator> f_hat_evaluator;
  std::shared_ptr<Evaluator> heuristic;
  std::shared_ptr<Evaluator> base_heuristic;
  std::shared_ptr<Evaluator> distance_heuristic;

  TLAs tlas;

  // Since the distribution is always the same for a given h-value, they
  // are cached in raw_beliefs, same for the post_expansion
  // distributions.
  PerStateInformation<ShiftedDistribution> beliefs;
  PerStateInformation<ShiftedDistribution> post_beliefs;
  std::vector<DiscreteDistribution*> raw_beliefs;
  std::vector<DiscreteDistribution*> raw_post_beliefs;

  // TODO: While backing up the beliefs for each state, we should also
  // back up the post search belief estimate at the same time I think,
  // and store it here.

  hstar_data_type<int> *hstar_data;
  hstar_data_type<long long> *post_expansion_belief_data;
  int hstar_gaussian_fallback_count;
  int post_expansion_belief_gaussian_fallback_count;

  // stores the index of the tla that owns the state
  // this is a hack to detect and prevent a state being expanded
  // under a tla when there is a different tla that has a shorter
  // path to that state.
  std::unordered_map<StateID, std::vector<int> > state_owners;

  // This is storage for applicable operators
  // It's kept here in the class because clearing a vector is
  // more efficient than creating a new one each iteration
  std::vector<OperatorID> applicables;
protected:
  void generate_tlas(GlobalState const &current_state);
  //std::unique_ptr<StateOpenList> create_open_list() const;
  double risk_analysis(std::size_t const alpha, const vector<ShiftedDistribution> &beliefs) const;
  std::size_t select_tla();
  void backup_beliefs();
  ShiftedDistribution node_belief(SearchNode const &);
  ShiftedDistribution post_expansion_belief(StateID best_state_id);
public:

  RiskLookaheadSearch(StateRegistry &state_registry,
                      int lookahead_bound,
                      std::shared_ptr<Evaluator> heuristic,
                      std::shared_ptr<Evaluator> base_heuristic,
                      std::shared_ptr<Evaluator> distance,
                      bool store_exploration_data,
                      ExpansionDelay *expansion_delay,
                      HeuristicError *heuristic_error,
                      hstar_data_type<int> *hstar_data,
                      hstar_data_type<long long> *post_expansion_belief_data);
  ~RiskLookaheadSearch() override;

  void initialize(const GlobalState &initial_state) override;
  auto search() -> SearchStatus override;

	void print_statistics() const override;

  RiskLookaheadSearch(const RiskLookaheadSearch &) = delete;
  RiskLookaheadSearch(RiskLookaheadSearch &&) = delete;

  auto operator=(const RiskLookaheadSearch &) = delete;
  auto operator=(RiskLookaheadSearch &&) = delete;

  auto get_beliefs() -> PerStateInformation<ShiftedDistribution> * { return &beliefs; }
  auto get_post_beliefs() -> PerStateInformation<ShiftedDistribution> * { return &post_beliefs; }
};

}


#endif
