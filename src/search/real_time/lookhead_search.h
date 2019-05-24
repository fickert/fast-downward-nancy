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
#include "DiscreteDistribution.h"

#include <memory>
#include <vector>

namespace real_time {

class HeuristicError;
class ExpansionDelay;

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

	ExpansionDelay *expansion_delay;
	std::unordered_map<StateID, int> open_list_insertion_time;

	HeuristicError *heuristic_error;

	void mark_expanded(SearchNode &node);

	bool check_goal_and_set_plan(const GlobalState &state);
public:
	LookaheadSearch(StateRegistry &state_registry,
	                int lookahead_bound,
	                bool store_exploration_data,
	                ExpansionDelay *expansion_delay,
	                HeuristicError *heuristic_error);
	virtual ~LookaheadSearch() = default;

	LookaheadSearch(const LookaheadSearch &) = delete;
	LookaheadSearch(LookaheadSearch &&) = delete;

	auto operator=(const LookaheadSearch &) = delete;
	auto operator=(LookaheadSearch &&) = delete;

	virtual void initialize(const GlobalState &initial_state);
	virtual auto search() -> SearchStatus = 0;

	virtual void print_statistics() const {}

	auto found_solution() const -> bool {return solution_found;}
	auto get_plan() const -> const Plan & {return plan;}
	auto get_statistics() const -> const SearchStatistics & {return *statistics;}

	auto get_search_space() const -> SearchSpace & { return *search_space; }
	auto get_frontier() const -> const decltype(frontier) & { return frontier; }
	auto get_predecessors() const -> const decltype(predecessors) & { return predecessors; }
	auto get_closed() const -> const decltype(closed) & { return closed; }

  // only implemented for lookahead search methods making use of distributions (risk)
  virtual auto get_beliefs() -> PerStateInformation<ShiftedDistribution> * { return nullptr; }
  virtual auto get_post_beliefs() -> PerStateInformation<ShiftedDistribution> * { return nullptr; }
};

class EagerLookaheadSearch : public LookaheadSearch {
	std::unique_ptr<StateOpenList> open_list;
protected:
	virtual auto create_open_list() const -> std::unique_ptr<StateOpenList> = 0;
public:
	EagerLookaheadSearch(StateRegistry &state_registry,
	                     int lookahead_bound,
	                     bool store_exploration_data,
	                     ExpansionDelay *expansion_delay,
	                     HeuristicError *heuristic_error);
	~EagerLookaheadSearch() override = default;

	EagerLookaheadSearch(const EagerLookaheadSearch &) = delete;
	EagerLookaheadSearch(EagerLookaheadSearch &&) = delete;

	auto operator=(const EagerLookaheadSearch &) = delete;
	auto operator=(EagerLookaheadSearch &&) = delete;

	void initialize(const GlobalState &initial_state) override;
	auto search() -> SearchStatus override;
};

class AStarLookaheadSearch : public EagerLookaheadSearch {
	std::shared_ptr<Evaluator> f_evaluator;
	std::shared_ptr<Evaluator> heuristic;
protected:
	auto create_open_list() const -> std::unique_ptr<StateOpenList> override;
public:
	AStarLookaheadSearch(StateRegistry &state_registry,
	                     int lookahead_bound,
	                     std::shared_ptr<Evaluator> heuristic,
	                     bool store_exploration_data,
	                     ExpansionDelay *expansion_delay,
	                     HeuristicError *heuristic_error);
	~AStarLookaheadSearch() override = default;

	AStarLookaheadSearch(const AStarLookaheadSearch &) = delete;
	AStarLookaheadSearch(AStarLookaheadSearch &&) = delete;

	auto operator=(const AStarLookaheadSearch &) = delete;
	auto operator=(AStarLookaheadSearch &&) = delete;
};

class BreadthFirstLookaheadSearch : public EagerLookaheadSearch {
protected:
	auto create_open_list() const->std::unique_ptr<StateOpenList> override;
public:
	BreadthFirstLookaheadSearch(StateRegistry &state_registry,
	                            int lookahead_bound,
	                            bool store_exploration_data,
	                            ExpansionDelay *expansion_delay,
	                            HeuristicError *heuristic_error);
	~BreadthFirstLookaheadSearch() override = default;

	BreadthFirstLookaheadSearch(const BreadthFirstLookaheadSearch &) = delete;
	BreadthFirstLookaheadSearch(BreadthFirstLookaheadSearch &&) = delete;

	auto operator=(const BreadthFirstLookaheadSearch &) = delete;
	auto operator=(BreadthFirstLookaheadSearch &&) = delete;
};

class FHatLookaheadSearch : public EagerLookaheadSearch {
	std::shared_ptr<Evaluator> f_hat_evaluator;
	std::shared_ptr<Evaluator> heuristic;
protected:
	auto create_open_list() const -> std::unique_ptr<StateOpenList> override;
public:
	FHatLookaheadSearch(StateRegistry &state_registry,
	                    int lookahead_bound,
	                    std::shared_ptr<Evaluator> heuristic,
	                    std::shared_ptr<Evaluator> distance,
	                    bool store_exploration_data,
	                    ExpansionDelay *expansion_delay,
	                    HeuristicError &heuristic_error);
	~FHatLookaheadSearch() override = default;

	FHatLookaheadSearch(const FHatLookaheadSearch &) = delete;
	FHatLookaheadSearch(FHatLookaheadSearch &&) = delete;

	auto operator=(const FHatLookaheadSearch &) = delete;
	auto operator=(FHatLookaheadSearch &&) = delete;
};

}

#endif
