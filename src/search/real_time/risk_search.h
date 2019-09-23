#ifndef RISK_SEARCH_H
#define RISK_SEARCH_H

#include "lookhead_search.h"
#include "DiscreteDistribution.h"
#include "belief_data.h"
#include "belief_store.h"
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

// This struct is used in the open list to sort nodes by expected
// value, with h as tie breaker.
struct NodeEvaluation
{
  double expected; // g + h_hat
  int g;
  int h;
  NodeEvaluation(double h_hat, int g, int h)
    : expected(h_hat + static_cast<double>(g)), g(g), h(h) {}
  ~NodeEvaluation() {}
};

struct TLAs
{
  using QueueEntry = std::pair<NodeEvaluation, StateID>;
  using QueueEntryCompare =
    struct {
      bool operator()(QueueEntry const &l, QueueEntry const &r) const
      {
       if (l.first.expected != r.first.expected) {
         return l.first.expected > r.first.expected;
       } else if (l.first.h != r.first.h) {
         return l.first.h > r.first.h;
       } else {
         return l.second.hash() < r.second.hash();
       }
      }
    };
  using Queue =
    std::priority_queue<QueueEntry,
                        std::vector<QueueEntry>,
                        QueueEntryCompare>;
  std::vector<OperatorID> ops;
  std::vector<int> op_costs;
  std::vector<Queue> open_lists;
  // backed up belief for each tla
  std::vector<ShiftedDistribution> beliefs;
  std::vector<ShiftedDistribution> post_beliefs;
  // "states" stores the current best state and h_hat in the frontier for each tla.
  // h_hat is used later in the decision strategy, but I don't need the entire distribution there.
  std::vector<std::pair<StateID, double> > states;
  std::vector<EvaluationContext> eval_contexts;

  GlobalState const *current_state;

  void reserve(std::size_t n);
  void clear();
  size_t size() const;
  QueueEntry remove_min(size_t tla_id);
  QueueEntry const &min(size_t tla_id) const;
};

class RiskLookaheadSearch : public LookaheadSearch
{
  using Beliefs = PerStateInformation<ShiftedDistribution>;

  std::shared_ptr<Evaluator> f_evaluator;
  std::shared_ptr<Evaluator> f_hat_evaluator;
  std::shared_ptr<Evaluator> heuristic;
  std::shared_ptr<Evaluator> base_heuristic;
  std::shared_ptr<Evaluator> distance_heuristic;

  TLAs tlas;

  // Since the distribution is always the same for a given feature
  // value, they are cached in raw_beliefs, same for the
  // post_expansion distributions.
  Beliefs beliefs;
  Beliefs post_beliefs;
  BeliefStore<int> raw_beliefs;
  BeliefStore<long long> raw_post_beliefs;
  ShiftedDistribution dead_end_belief;
  HStarData<int> *hstar_data;
  HStarData<long long> *post_expansion_belief_data;

  int hstar_gaussian_fallback_count;
  int post_expansion_belief_gaussian_fallback_count;
  int alpha_expansion_count;
  int beta_expansion_count;

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
  //std::unique_ptr<StateOpenList> create_open_list() const;
  double risk_analysis(std::size_t const alpha, const vector<ShiftedDistribution> &beliefs) const;
  std::size_t select_tla();
  void backup_beliefs();
  ShiftedDistribution get_belief(EvaluationContext &);
  ShiftedDistribution get_post_belief(StateID best_state_id);
public:

  RiskLookaheadSearch(StateRegistry &state_registry,
                      int lookahead_bound,
                      std::shared_ptr<Evaluator> heuristic,
                      std::shared_ptr<Evaluator> base_heuristic,
                      std::shared_ptr<Evaluator> distance,
                      bool store_exploration_data,
                      ExpansionDelay *expansion_delay,
                      HeuristicError *heuristic_error,
                      HStarData<int> *hstar_data,
                      HStarData<long long> *post_expansion_belief_data,
                      SearchEngine const *search_engine,
                      DataFeatureKind f_kind,
                      DataFeatureKind pf_kind);
  ~RiskLookaheadSearch() override;

  void initialize(const GlobalState &initial_state) override;
  auto search() -> SearchStatus override;

	void print_statistics() const override;

  RiskLookaheadSearch(const RiskLookaheadSearch &) = delete;
  RiskLookaheadSearch(RiskLookaheadSearch &&) = delete;

  auto operator=(const RiskLookaheadSearch &) = delete;
  auto operator=(RiskLookaheadSearch &&) = delete;

  auto get_tlas() -> TLAs const * { return &tlas; }
  auto get_beliefs() -> PerStateInformation<ShiftedDistribution> * { return &beliefs; }
  auto get_post_beliefs() -> PerStateInformation<ShiftedDistribution> * { return &post_beliefs; }
};

}


#endif
