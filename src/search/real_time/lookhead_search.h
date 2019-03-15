#ifndef REAL_TIME_LOOKAHEAD_SEARCH_H
#define REAL_TIME_LOOKAHEAD_SEARCH_H

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

namespace real_time {

class LookaheadSearch {
	bool solution_found;
	Plan plan;
protected:
	const std::shared_ptr<AbstractTask> task;
	TaskProxy task_proxy;
	StateRegistry &state_registry;
	const successor_generator::SuccessorGenerator &successor_generator;
	std::unique_ptr<SearchStatistics> statistics;

	std::unique_ptr<SearchSpace> search_space;
	const int lookahead_bound;
	std::vector<StateID> frontier;

	const bool store_exploration_data;
	std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy>>> predecessors;
	std::unordered_set<StateID> closed;

	bool check_goal_and_set_plan(const GlobalState &state);
public:
	LookaheadSearch(StateRegistry &state_registry,
	                int lookahead_bound,
	                bool store_exploration_data);
	virtual ~LookaheadSearch() = default;

	LookaheadSearch(const LookaheadSearch &) = delete;
	LookaheadSearch(LookaheadSearch &&) = delete;

	auto operator=(const LookaheadSearch &) = delete;
	auto operator=(LookaheadSearch &&) = delete;

	virtual void initialize(const GlobalState &initial_state);
	virtual auto search() -> SearchStatus = 0;

	auto found_solution() const -> bool {return solution_found;}
	auto get_plan() const -> const Plan & {return plan;}
	auto get_statistics() const -> const SearchStatistics & {return *statistics;}

	auto get_search_space() const -> SearchSpace & { return *search_space; }
	auto get_frontier() const -> const decltype(frontier) & { return frontier; }
	auto get_predecessors() const -> const decltype(predecessors) & { return predecessors; }
	auto get_closed() const -> const decltype(closed) & { return closed; }
};

class AStarLookaheadSearch : public LookaheadSearch {
	std::shared_ptr<Evaluator> heuristic;
	std::shared_ptr<Evaluator> f_evaluator;

	std::unique_ptr<StateOpenList> open_list;
public:
	AStarLookaheadSearch(StateRegistry &state_registry,
	                     int lookahead_bound,
	                     std::shared_ptr<Evaluator> heuristic,
	                     bool store_exploration_data);
	~AStarLookaheadSearch() override = default;

	AStarLookaheadSearch(const AStarLookaheadSearch &) = delete;
	AStarLookaheadSearch(AStarLookaheadSearch &&) = delete;

	auto operator=(const AStarLookaheadSearch &) = delete;
	auto operator=(AStarLookaheadSearch &&) = delete;

	void initialize(const GlobalState &initial_state) override;
	auto search() -> SearchStatus override;
};

}

#endif
