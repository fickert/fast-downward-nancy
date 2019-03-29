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

static constexpr auto MAX_SAMPLES = 100;

struct hstar_sample {
	int hstar_value;
	int count;
};

struct hstar_data_entry {
	int value_count;
	std::vector<hstar_sample> hstar_values;
};

using hstar_data_type = std::unordered_map<int, hstar_data_entry>;

inline auto read_hstar_data(const std::string &file_name) -> hstar_data_type {
	auto parsed_hstar_data = hstar_data_type();
	std::ifstream f(file_name);
	std::string line;
	int h, valueCount, hs, hsCount;

	while (std::getline(f, line)) {
		std::stringstream ss(line);
		ss >> h;
		ss >> valueCount;

		if (parsed_hstar_data.find(h) != std::end(parsed_hstar_data)) {
			std::cerr << "error: duplicate h from data " << h << std::endl;
			utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
		}

		auto &hstar_data_for_h = parsed_hstar_data.emplace(h, hstar_data_entry{valueCount, {}}).first->second.hstar_values;
		while (!ss.eof()) {
			ss >> hs;
			ss >> hsCount;
			hstar_data_for_h.emplace_back(hstar_sample{hs, hsCount});
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
