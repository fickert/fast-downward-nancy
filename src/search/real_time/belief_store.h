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

  BeliefStore(DataFeatureKind kind, HStarData<CountT> *data)
    : data(data)
  {
    raws.emplace_back();
    raws.back().features.emplace_back(goal_feature(kind));
    raws.back().distributions.push_back(new DiscreteDistribution(1, 0.0));
  }

  ~BeliefStore() {}

  void resize(size_t s) { raws.resize(s); }

  size_t size() const { return raws.size(); }

  void remember(BeliefEntries &bin, DataFeature const &df, DiscreteDistribution *d)
  {
    bin.features.emplace_back(df);
    bin.distributions.push_back(d);
  }

  void remember(DataFeature const &df, DiscreteDistribution *d)
  {
    assert(raws.size() >= static_cast<size_t>(df.h));
    auto &bin = raws[df.h];
    remember(bin, df, d);
  }

  // This is the key function to get a fresh belief distribution based
  // on some feature.  It takes care of the following cases:
  // - If the distribution for this feature has been computed before,
  //   then we have cached it, and can simply return it.
  // - Else if we have data for this feature, we make the
  //   distribution now, cache it, and return it.
  // - Else we look for the next lower h value for which we have data.
  //   In the bucket for this h, we look for the feature that matches
  //   the input most closely.  The distribution for this closest
  //   match is copied, extrapolated, and cached for the input
  //   feature.
  // - If no data is available in the first place, we return null
  //   here.
  DiscreteDistribution *get_distribution(DataFeature const &df_in)
  {
    DiscreteDistribution *res = nullptr;
    DiscreteDistribution *raw;
    int h_in = df_in.h;

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
      return nullptr;
    }
    auto const &data_ref = *data;

    // else check if we have data for this exact feature
    // configuration.
    if (static_cast<size_t>(h_in) < data_ref.size()) {
      auto in_data = vec_find(data_ref[h_in].features, df_in);
      if (in_data.second) {
        assert(in_data.first < data_ref[h_in].values.size());
        assert(data_ref[h_in].features.size() == data_ref[h_in].values.size());
        raw = new DiscreteDistribution(MAX_SAMPLES,
                                       data_ref[h_in].values[in_data.first]);
        remember(bin, df_in, raw);
        return raw;
      }
    }

    // else go to the first h below for which we have data, and take the
    // closest match.  if we have that distribution already, use it,
    // otherwise compute it here.
    int h_adj = h_in;
    if (static_cast<size_t>(h_in) >= data_ref.size())
      h_adj = data_ref.size() - 1;
    while (h_adj >= 0) {
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

        // make the distribution for the closest match if we don't have
        // it already.
        auto adj_in_raws = vec_find(adj_bin.features,min_df);
        if (adj_in_raws.second) {
          raw = adj_bin.distributions[adj_in_raws.first];
        } else {
          raw = new DiscreteDistribution(MAX_SAMPLES, data_bin.values[min_idx]);
          remember(adj_bin, min_df, raw);
        }

        // copy the distribution for the closest match.
        res = new DiscreteDistribution(raw, shift);
        remember(bin, df_in, res);
        break;
      } else {
        --h_adj;
      }
    }

    assert(res != nullptr);
    return res;
  }
};

}

#endif
