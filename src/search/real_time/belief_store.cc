#include "belief_store.h"

#define TRACKBLF

#ifdef TRACKBLF
#define BEGINF(X) std::cout << "BLF: ENTER: " << X << "\n";
#define ENDF(X) std::cout << "BLF: EXIT: " << X << "\n";
#define TRACKP(X) std::cout << "BLF: " << X << "\n";
#else
#define BEGINF(X)
#define ENDF(X)
#define TRACKP(X)
#endif

namespace real_time
{

template<typename CountT>
BeliefStore<CountT>::BeliefStore(DataFeatureKind kind, HStarData<CountT> *data)
	: data(data)
{
	raws.emplace_back();
	raws.back().features.emplace_back(goal_feature(kind));
	raws.back().distributions.push_back(new DiscreteDistribution(1, 0.0));

	if (nullptr == data)
		return;

	for (HStarEntries<CountT> const &entries : data->data) {
		for (size_t i = 0; i < entries.size(); ++i) {
			int h = entries.features[i].h;
			ensure_size(h+1);
			auto &hbin = raws[h];
			hbin.features.push_back(entries.features[i]);
			hbin.distributions.push_back(new DiscreteDistribution(MAX_SAMPLES, entries.values[i]));
		}
	}
}

template<typename CountT>
BeliefStore<CountT>::~BeliefStore() {}

template<typename CountT>
void BeliefStore<CountT>::ensure_size(size_t s)
{
	if (s >= raws.size()) raws.resize(s+1);
}

template<typename CountT>
size_t BeliefStore<CountT>::size() const
{
	return raws.size();
}


template<typename CountT>
void BeliefStore<CountT>::remember(BeliefEntries &bin, DataFeature const &df, DiscreteDistribution *d)
{
	bin.features.emplace_back(df);
	bin.distributions.push_back(d);
}

template<typename CountT>
void BeliefStore<CountT>::remember(DataFeature const &df, DiscreteDistribution *d)
{
	assert(raws.size() >= static_cast<size_t>(df.h));
	auto &bin = raws[df.h];
	remember(bin, df, d);
}


// TODO: the comment here isn't right anymore, since we don't
// do this lazy distribution construction anymore.

// This is the function to get a fresh belief distribution based
// on some feature.  It takes care of the following cases:
// - If the distribution for this feature has been computed before,
//   then we have cached it, and can simply return it.
// - Else we know we don't have an exact data match.  So we look for
//   the next lower h value for which we have data.  In the bucket for
//   this h, we look for the feature that matches the input most
//   closely.  The distribution for this closest match is copied,
//   extrapolated, and cached for the input feature.
// - If no data is available in the first place, we return null
//   here.
template<typename CountT>
DiscreteDistribution *BeliefStore<CountT>::get_distribution(DataFeature const &df_in)
{
	BEGINF(__func__);
	DiscreteDistribution *res = nullptr;
	DiscreteDistribution *raw;
	int h_in = df_in.h;

	assert(h_in >= 0);
	ensure_size(static_cast<size_t>(h_in));
	if (!(raws.size() > static_cast<size_t>(h_in))) {
		std::cout << raws.size() << ", " << h_in << "\n";
	}
	assert(raws.size() > static_cast<size_t>(h_in));
	BeliefEntries &bin = raws[h_in];

	// first check if we have this distribution already.
	auto in_raws = vec_find(bin.features, df_in);
	if (in_raws.second) {
		return bin.distributions[in_raws.first];
	}

	// check if we even have data.  if we don't, we can't go on here,
	// and the search has to use the gauss fallback.
	if (data == nullptr) {
		ENDF(__func__);
		return nullptr;
	}
	auto const &data_ref = *data;

	// else we know we don't have exact data for this
	// feature.  So we go to the first h below for which
	// we have data, and take the closest match.
	TRACKP("didn't find match for data feature " << df_in);
	int h_adj = h_in;
	if (static_cast<size_t>(h_in) >= data_ref.size())
		h_adj = data_ref.size() - 1;
	TRACKP("h_adj " << h_adj);
	while (h_adj >= 0) {
		TRACKP("trying adjusted h " << h_adj);
		auto const &data_bin = data_ref[h_adj];
		if (!data_bin.empty()) {
			// among all feature values for which we have data, find the
			// closest match
			int min_diff = std::numeric_limits<int>::max();
			size_t min_idx = 0;
			for (size_t i = 0; i < data_bin.features.size(); ++i) {
				int diff = df_in - data_bin.features[i];
				if (diff < min_diff) {
					min_diff = diff;
					min_idx = i;
				}
			}

			assert(min_idx < data_bin.features.size());
			assert(static_cast<size_t>(h_adj) < raws.size());
			DataFeature const &min_df = data_bin.features[min_idx];
			auto &adj_bin = raws[h_adj];
			int const shift = h_in - h_adj;
			assert(shift >= 0);

			auto adj_in_raws = vec_find(adj_bin.features,min_df);
			assert(adj_in_raws.second);
			raw = adj_bin.distributions[adj_in_raws.first];

			// copy the distribution for the closest match.
			res = new DiscreteDistribution(raw, shift);
			remember(bin, df_in, res);
			break;
		} else {
			--h_adj;
		}
	}

	assert(res != nullptr);
	ENDF(__func__);
	return res;
}

template class BeliefStore<int>;
template class BeliefStore<long long>;

}

#undef BEGINF
#undef ENDF
#undef TRACKP
