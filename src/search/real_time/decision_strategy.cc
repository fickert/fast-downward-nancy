#include "decision_strategy.h"

#include "../plan_manager.h"
#include "../search_space.h"
#include "risk_search.h"

#include <algorithm>

namespace real_time {

ScalarDecisionStrategy::ScalarDecisionStrategy(const StateRegistry &state_registry, decltype(evaluator) evaluator)
	: state_registry(state_registry),
	  evaluator(evaluator) {}

auto ScalarDecisionStrategy::get_top_level_action(const std::vector<StateID> &frontier, SearchSpace &search_space) const -> OperatorID {
	assert(!frontier.empty());
	const auto best_state_id = std::min_element(std::begin(frontier), std::end(frontier), [this, &search_space](const auto &lhs, const auto &rhs) {
		return evaluator(lhs, search_space) < evaluator(rhs, search_space);
	});
	auto plan = Plan();
	search_space.trace_path(state_registry.lookup_state(*best_state_id), plan);
	assert(!plan.empty());
	return plan.front();
}


NancyDecisionStrategy::NancyDecisionStrategy(TLAs const *tlas,
                                             SearchEngine const &engine)
  : tlas(tlas), engine(engine)
{
  assert(tlas);
}

// beliefs from frontier are already backed up to tlas during expansion.
// Now we just have to select the minimum expected among them.
OperatorID NancyDecisionStrategy::pick_top_level_action(SearchSpace const &search_space)
{
  //std::cout << "pick_top_level_action()\n";
  if (target_path.empty()) {
    target_h_hat = std::numeric_limits<double>::infinity();
    target_f_hat = std::numeric_limits<double>::infinity();
  }

  // TODO: replace double == comparisons with something more appropriate

  double min_h = target_h_hat;
  double min_f = target_f_hat;
  // std::cout << "current target f, h: " << target_f_hat << ", " << target_h_hat << "\n";
  size_t min_id = 0;
  bool new_direction = false;
  //std::cout << "tla values in decision: ";
  for (size_t i = 0; i < tlas->size(); ++i) {
    const double cur = tlas->beliefs[i].expected_cost();
    if (std::numeric_limits<double>::infinity() == cur)
      continue;

    const double tla_f_hat = cur + static_cast<double>(tlas->op_costs[i]);
    //std::cout << "(" << cur << ", " << static_cast<double>(tlas->op_costs[i]) << "), ";
    if (tla_f_hat > min_f) {
      continue;
    } else {
      const double frontier_h_hat = tlas->states[i].second;
      if (tla_f_hat == min_f) {
        if (frontier_h_hat >= min_h) {
          continue;
        } else {
          new_direction = true;
          min_id = i;
          min_h = frontier_h_hat;
        }
      } else {
        assert(tla_f_hat < min_f);
        // tla_f_hat < target_f_hat
        new_direction = true;
        min_id = i;
        min_h = frontier_h_hat;
        min_f = tla_f_hat;
      }
    }
  }
  //std::cout << "\n";

  if (new_direction && !target_path.empty()) {
    OperatorID next_action = target_path.back();
    new_direction = next_action != tlas->ops[min_id];
  }

  if (new_direction) {
    // std::cout << "new direction\n";
    // build new path
    target_path.clear();
    // std::cout << "tracing new path\n";
    // TODO: could trace only to successor of initial_id to save the pop_back below
    search_space.trace_path_rev_id_to(tlas->current_state->get_id(), tlas->states[min_id].first, target_path);
    assert(!target_path.empty());
    OperatorID next_action = target_path.back();
    assert(std::find(tlas->ops.begin(), tlas->ops.end(), next_action) != tlas->ops.end());
    target_path.pop_back();
    target_h_hat = min_h;
    target_f_hat = min_f;
    //std::cout << "new target state " << tlas->states[min_id].first << " with f, h: " << target_f_hat << ", " << target_h_hat << "\n";
    return next_action;
  } else {
    // std::cout << "follow path\n";
    assert(!target_path.empty());
    OperatorID next_action = target_path.back();
    assert(std::find(tlas->ops.begin(), tlas->ops.end(), next_action) != tlas->ops.end());
    target_path.pop_back();
    if (min_h < target_h_hat)
      target_h_hat = min_h;
    target_f_hat -= engine.get_adjusted_cost(engine.get_operators()[next_action]);
    return next_action;
  }
}

}
