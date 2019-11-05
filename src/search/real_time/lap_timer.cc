#include "lap_timer.h"

namespace real_time
{

LapTimer::LapTimer()
	: cache(1)
{
}

LapTimer::~LapTimer()
{
}

void LapTimer::reset()
{
	tp = std::chrono::system_clock::now();
}

std::chrono::milliseconds LapTimer::pause()
{
	auto const n = std::chrono::system_clock::now();
	cache = std::chrono::duration_cast<std::chrono::milliseconds>(n - tp);
	return cache;
}

std::chrono::milliseconds LapTimer::get() const
{
	return cache;
}

}
