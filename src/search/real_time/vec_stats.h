#ifndef REAL_TIME_VEC_STATS_H
#define REAL_TIME_VEC_STATS_H

#include <vector>
#include <numeric>

namespace real_time
{

template<typename T>
struct VecStats
{
	T avg, min, max, med;
};

template<typename T>
VecStats<T> vec_stats(std::vector<T> const &v)
{
	assert(std::is_sorted(v.begin(), v.end()));
	const auto s = static_cast<long>(v.size());
	assert(s > 0);
	VecStats<T> r;
	r.avg = std::accumulate(v.begin(), v.end(), 0) / s;
	r.min = v[0];
	r.max = v[s-1];
	r.med = ((s & 1) == 0) ? (v[(s/2)-1] + v[s/2]) / 2 : v[s/2];
	return r;
}

}

#endif
