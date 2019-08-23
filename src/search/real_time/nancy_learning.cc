#include "../state_registry.h"
#include "nancy_learning.h"

#include <queue>
#include <iostream>

namespace real_time
{

NancyLearning::NancyLearning(StateRegistry const &state_registry,
                             SearchEngine const *search_engine,
                             Beliefs *beliefs,
                             Beliefs *post_beliefs)
	: state_registry(state_registry), search_engine(search_engine), beliefs(beliefs), post_beliefs(post_beliefs)
{
}

void NancyLearning::apply_updates(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors,
                                  const std::vector<StateID> &frontier,
                                  const std::unordered_set<StateID> &closed,
                                  GlobalState const &current_state) const
{
  auto closed_copy = closed;
  apply_updates(predecessors, frontier, std::move(closed_copy), current_state);
}


using LearningQueueEntry = std::pair<ShiftedDistribution, StateID>;

struct LearningQueueComp
{
  bool operator()(const LearningQueueEntry &a, const LearningQueueEntry &b) const
  {
    return a.first.expected_cost() >  b.first.expected_cost();
  }
};

void NancyLearning::apply_updates(const std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> &predecessors,
                                  const std::vector<StateID> &frontier,
                                  std::unordered_set<StateID> &&closed,
                                  GlobalState const &current_state) const
{
	auto learning_queue = std::priority_queue<LearningQueueEntry, std::vector<LearningQueueEntry>, LearningQueueComp>();
  auto initial_id = current_state.get_id();

  // this stores the running expected values for dijkstra backup
  // std::unordered_map<StateID, double> exp_cache;

  for (const auto &state_id : closed) {
    auto const &state = state_registry.lookup_state(state_id);
    // exp_cache[state_id] = (*beliefs)[state].expected_value;
    (*beliefs)[state].expected_value = std::numeric_limits<double>::infinity();
  }

  for (const auto &state_id : frontier) {
    auto const &state = state_registry.lookup_state(state_id);
    learning_queue.emplace((*beliefs)[state], state_id);
    // exp_cache[state_id] = (*beliefs)[state].expected_cost();
    closed.erase(state_id);
  }

  while (!learning_queue.empty()) {
    auto &top = learning_queue.top();
    ShiftedDistribution dstr = top.first;
    StateID state_id = top.second;
    learning_queue.pop();
    auto const &state = state_registry.lookup_state(state_id);
    assert((*beliefs)[state].distribution != nullptr);
    // if (dstr.expected_cost() != (*beliefs)[state].expected_cost()) {
    //   continue;
    // }
    if (dstr.expected_value == std::numeric_limits<double>::infinity()) {
      continue;
    }

    auto cls = closed.find(state_id);

    if (cls != closed.end()) {
      closed.erase(cls);
    }

    auto preds = predecessors.find(state_id);
    if (preds == predecessors.end()) {
      continue;
    }

    for (auto const &[p_id, op] : preds->second) {
      auto cls = closed.find(p_id);
      if (cls == closed.end()) {
        continue;
      }

      auto const &predecessor = state_registry.lookup_state(p_id);
      //assert(p_exp_cached >= 0.0);
      auto const new_exp = dstr.expected_cost() + search_engine->get_adjusted_cost(op);
      ShiftedDistribution &p_belief = (*beliefs)[predecessor];
      auto const p_exp = p_belief.expected_cost();
      if (p_exp > new_exp) {
        // to be clear here:
        // - the belief is only backed up if its expected value increased
        //   compared to before.
        // - the new belief that's stored for the predecessor is
        //   1. the same raw distribution as the current state.
        //   2. a shift given by op cost + the shift for the current state.
        // - the new expected value for the predecessor will be the
        //   current state's expected value + the operator cost which is
        //   the same as the expected value of the raw distribution + shift
        //closed.erase(cls);

        // only back up if the value increased (strictly increasing
        // for initial state to prevent running in cycles)
        // auto const p_exp_cached = exp_cache[p_id];
        // bool const should_learn =
        //      (p_id == initial_id && new_exp > p_exp_cached)
        //   || (p_id != initial_id && new_exp >= p_exp_cached);

        bool const should_learn = true;
        if (should_learn) {
          // backup the main belief
          p_belief.set_and_shift(dstr, search_engine->get_adjusted_cost(op));
          // std::cout << "state " << p_id << " gets belief with exp " << new_exp << " from " << state_id << "\n";

          // backup the post expansion belief
          ShiftedDistribution &p_post_belief = (*post_beliefs)[predecessor];
          ShiftedDistribution &s_post_belief = (*post_beliefs)[state];
          assert(dstr.shift == s_post_belief.shift);
          assert(s_post_belief.distribution);
          p_post_belief.set_and_shift(s_post_belief, search_engine->get_adjusted_cost(op));
          assert(std::abs(new_exp - p_belief.expected_cost()) < 0.001);
          // learning_queue.emplace(p_belief, p_id);
        }

        learning_queue.emplace(p_belief, p_id);
      }
    }
  }

  // auto const &initial_state = state_registry.lookup_state(initial_id);
  // if ((*beliefs)[initial_state].expected_value == exp_cache[initial_id]) {
  //   (*beliefs)[initial_state].shift++;
  //   (*beliefs)[initial_state].expected_value++;
  // }

  // auto const &initial_state = state_registry.lookup_state(initial_id);
  // std::cout << "Learning changed h_hat of current state from " << exp_cache[initial_id] << " to " << (*beliefs)[initial_state].expected_cost() << "\n";
}
}
