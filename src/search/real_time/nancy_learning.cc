#include "../state_registry.h"
#include "nancy_learning.h"

#include <queue>
#include <iostream>

namespace real_time
{

NancyLearning::NancyLearning(StateRegistry const &state_registry)
	: state_registry(state_registry)
{
}

void NancyLearning::apply_updates(const std::unordered_map<StateID,
                                  std::vector<std::pair<StateID, OperatorProxy>>> &predecessors,
                                  const std::vector<StateID> &frontier,
                                  const std::unordered_set<StateID> &closed,
                                  PerStateInformation<ShiftedDistribution> *beliefs) const
{
  auto closed_copy = closed;
  apply_updates(predecessors, frontier, std::move(closed_copy), beliefs);
}


using LearningQueueEntry = std::pair<ShiftedDistribution, StateID>;
struct ShiftedDistributionComp
{
  bool operator()(const ShiftedDistribution &a, const ShiftedDistribution &b) const
  {
    return a.expected_cost() < b.expected_cost();
  }
};

struct LearningQueueComp
{
  bool operator()(const LearningQueueEntry &a, const LearningQueueEntry &b) const
  {
    return ShiftedDistributionComp()(a.first, b.first);
  }
};

void NancyLearning::apply_updates(const std::unordered_map<StateID,
                                  std::vector<std::pair<StateID,
                                  OperatorProxy>>> &predecessors, const std::vector<StateID> &frontier,
                                  std::unordered_set<StateID> &&closed,
                                  PerStateInformation<ShiftedDistribution> *beliefs) const
{
	auto learning_queue = std::priority_queue<LearningQueueEntry, std::vector<LearningQueueEntry>, LearningQueueComp>();

  for (const auto &state_id : closed) {
    auto const &state = state_registry.lookup_state(state_id);
    (*beliefs)[state].expected_value = std::numeric_limits<double>::infinity();
  }

  for (const auto &state_id : frontier) {
    auto const &state = state_registry.lookup_state(state_id);
    learning_queue.emplace((*beliefs)[state], state_id);
    closed.erase(state_id);
  }

  while (!learning_queue.empty()) {
    auto &top = learning_queue.top();
    ShiftedDistribution dstr = top.first;
    StateID state_id = top.second;
    learning_queue.pop();
    auto const &state = state_registry.lookup_state(state_id);
    assert((*beliefs)[state].distribution != nullptr);

    if (dstr.expected_value == std::numeric_limits<double>::infinity()) {
      continue;
    }

    auto preds = predecessors.find(state_id);
    if (preds == predecessors.end()) {
      continue;
    }

    for (auto const &[p_id, op] : preds->second) {
      auto cls = closed.find(p_id);
      if (closed.find(p_id) == closed.end()) {
        continue;
      }
      auto const &predecessor = state_registry.lookup_state(p_id);
      auto const p_exp = (*beliefs)[predecessor].expected_cost();
      auto const new_exp = dstr.expected_cost() + op.get_cost();
      if (p_exp > new_exp) {
        // to be clear here:
        // - the new belief that's stored for the predecessor is
        //   1. the same raw distribution as the current state.
        //   2. a shift given by op cost + the shift for the current state.
        // - the new expected value for the predecessor will be the
        //   current state's expected value + the operator cost which is
        //   the same as the expected value of the raw distribution + shift
        closed.erase(cls);
        ShiftedDistribution &p_belief = (*beliefs)[predecessor];
        p_belief.set(dstr.distribution, dstr.shift + op.get_cost());
        assert(std::abs(new_exp - p_belief.expected_cost()) < 0.001);
        learning_queue.emplace(p_belief, p_id);
      }
    }
  }
}
}
