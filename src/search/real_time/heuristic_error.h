#ifndef REAL_TIME_HEURISTIC_ERROR_H
#define REAL_TIME_HEURISTIC_ERROR_H

#include "../evaluator.h"
#include "../search_space.h"
#include "../state_id.h"

#include <memory>

namespace real_time {

class HeuristicError {
	const StateRegistry &state_registry;
	std::shared_ptr<Evaluator> f_evaluator;
	std::shared_ptr<Evaluator> heuristic;
	std::shared_ptr<Evaluator> distance;

	double average_heuristic_error;
	double average_distance_error;
	int count;

	StateID current_state_id;
	StateID best_successor_id;
	int best_successor_op_cost;
	int best_successor_value;

public:
	HeuristicError(const StateRegistry &state_registry, std::shared_ptr<Evaluator> heuristic, std::shared_ptr<Evaluator> distance);

	void set_expanding_state(const GlobalState &state);
	void add_successor(const SearchNode &successor_node, int op_cost);
	void update_error();

	auto get_average_heuristic_error() const -> double { return average_heuristic_error; }
	auto get_average_distance_error() const -> double { return average_distance_error; }
};



}

#endif
