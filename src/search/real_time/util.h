#ifndef REAL_TIME_UTIL_H
#define REAL_TIME_UTIL_H

#include "debiased_heuristic.h"
#include "heuristic_error.h"
#include "../evaluator.h"
#include "../evaluators/g_evaluator.h"
#include "../evaluators/sum_evaluator.h"

#include <memory>
#include <fstream>
#include <sstream>

namespace real_time {

enum Constants
{
 MAX_SAMPLES = 64,
};

template<class count_type = int>
struct hstar_sample {
	int hstar_value;
	count_type count;
};

template<class count_type = int>
struct hstar_data_entry {
	count_type value_count;
	std::vector<hstar_sample<count_type>> hstar_values;
};

template<class count_type = int>
using hstar_data_type = std::unordered_map<int, hstar_data_entry<count_type>>;

template<class count_type = int>
auto read_hstar_data(const std::string &file_name) -> hstar_data_type<count_type> {
	auto parsed_hstar_data = hstar_data_type<count_type>();
	std::ifstream f(file_name);
	std::string line;
	int h, hs;
	count_type valueCount, hsCount;

  // TODO: this is actually worth optimizing since we have lots of
  // data. in my medium sized transport test instance, perf reports
  // that 15% of the time is spent parsing (in calls to
  // std::istream::operator>>, std::num_get<char>, and other istream
  // functions).  a custom parser would probably be more efficient.
	while (std::getline(f, line)) {
		std::stringstream ss(line);
		ss >> h;
		ss >> valueCount;

		if (parsed_hstar_data.find(h) != std::end(parsed_hstar_data)) {
			std::cerr << "error: duplicate h from data " << h << std::endl;
			utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
		}

		auto &hstar_data_for_h = parsed_hstar_data.emplace(h, hstar_data_entry<count_type>{valueCount, {}}).first->second.hstar_values;
		while (!ss.eof()) {
			ss >> hs;
			ss >> hsCount;
			hstar_data_for_h.emplace_back(hstar_sample<count_type>{hs, hsCount});
		}
	}
	f.close();
	return parsed_hstar_data;
}

inline auto create_f_hat_evaluator(std::shared_ptr<Evaluator> heuristic, std::shared_ptr<Evaluator> distance, HeuristicError &heuristic_error) -> std::shared_ptr<Evaluator> {
	const auto h_hat_evaluator = std::make_shared<DebiasedHeuristic>(heuristic, distance, heuristic_error);
	const auto g_evaluator = std::make_shared<g_evaluator::GEvaluator>();
	return std::make_shared<sum_evaluator::SumEvaluator>(std::vector<std::shared_ptr<Evaluator>>{h_hat_evaluator, g_evaluator});
}

}

#endif
