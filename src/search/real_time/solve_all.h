#ifndef REAL_TIME_SOLVE_ALL_H
#define REAL_TIME_SOLVE_ALL_H

#include "../options/options.h"
#include "../open_list.h"
#include "../search_engine.h"
#include "../utils/countdown_timer.h"

#include <memory>
#include <vector>
#include <unordered_set>
#include <unordered_map>

class Evaluator;
class PruningMethod;

namespace solve_complete
{

struct StateOpHash
{
  std::size_t operator()(std::pair<StateID, OperatorProxy> const &p) const
  {
    return p.first.hash();
  }
};

class SolveAll : public SearchEngine
{
  std::unique_ptr<StateOpenList> open_list;
  const bool reopen_closed_nodes;
  std::shared_ptr<Evaluator> evaluator;
  StateID initial_id;
  std::vector<StateID> goals;
  std::unordered_map<StateID, int> hs;
  std::unordered_map<StateID, int> hstars;
  //std::unordered_map<StateID, std::unordered_set<std::pair<StateID, OperatorProxy>, StateOpHash> > preds;
  std::unordered_map<StateID, std::vector<std::pair<StateID, OperatorProxy> > > preds;
  std::unordered_map<StateID, std::pair<StateID, OperatorProxy> > best_succ;
	std::unordered_set<StateID> expanded_states;
  // std::unordered_map<StateID, std::pair<int, int>> solved_states;
  std::vector<OperatorID> initial_plan;
  std::queue<std::pair<StateID,int> > d_queue;
  bool planning;
  const double reserved_time;
	utils::CountdownTimer timer;
	const std::string hstar_file;
  const std::string successors_file;

	void dump_hstar_values() const;
	void compute_and_dump_successors_data();
  void backup_hstar(StateID const s, int const acc = 0);
  std::unique_ptr<StateOpenList> create_open_list();
  // void start_f_value_statistics(EvaluationContext &eval_context);
  // void update_f_value_statistics(const SearchNode &node);
  // void reward_progress();
  void print_checkpoint_line(int g) const;

protected:
  virtual void initialize() override;
  virtual SearchStatus step() override;

public:
  explicit SolveAll(const options::Options &opts);
  virtual ~SolveAll() = default;

  virtual void print_statistics() const override;

  void dump_search_space() const;
};
}

#endif
