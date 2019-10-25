#ifndef REAL_TIME_BELIEF_STORE_H
#define REAL_TIME_BELIEF_STORE_H

#include <vector>

#include "belief_data.h"
#include "DiscreteDistribution.h"


namespace real_time
{

struct BeliefEntries
{
	std::vector<DataFeature> features;
	std::vector<DiscreteDistribution*> distributions;
	BeliefEntries() {}
	~BeliefEntries() {}
};

template<typename CountT = int>
struct BeliefStore
{
	std::vector<BeliefEntries> raws;
	HStarData<CountT> const *data;

	BeliefStore(DataFeatureKind kind, HStarData<CountT> *data);
	~BeliefStore();

	void ensure_size(size_t s);
	size_t size() const;

	void remember(BeliefEntries &bin, DataFeature const &df, DiscreteDistribution *d);
	void remember(DataFeature const &df, DiscreteDistribution *d);

	DiscreteDistribution *get_distribution(DataFeature const &df_in);
};

}


#endif
