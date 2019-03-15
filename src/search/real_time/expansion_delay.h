#ifndef REAL_TIME_EXPANSION_DELAY_H
#define REAL_TIME_EXPANSION_DELAY_H

#include <deque>

class SearchStatistics;

namespace real_time {

class ExpansionDelay {
	double avg_expansion_delay;
	int total_expansions;

	std::deque<int> last_delays;
	long long last_delays_sum;
	const std::size_t moving_average_size;
public:
	explicit ExpansionDelay(std::size_t moving_average_size);

	void update_expansion_delay(int delay);
	auto get_avg_expansion_delay() const -> double;
};

}

#endif
