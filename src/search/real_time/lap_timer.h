#ifndef REAL_TIME_LAP_TIMER_H
#define REAL_TIME_LAP_TIMER_H

#include <chrono>

namespace real_time
{

struct LapTimer
{
	std::chrono::time_point<std::chrono::system_clock> tp;
	std::chrono::milliseconds cache;

	LapTimer();
	~LapTimer();

	void reset();
	std::chrono::milliseconds pause();
	std::chrono::milliseconds get() const;
};

}

#endif
