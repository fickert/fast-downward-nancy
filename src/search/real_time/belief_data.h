#ifndef REAL_TIME_BELIEF_DATA_H
#define REAL_TIME_BELIEF_DATA_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <cassert>
#include <vector>

#include "../utils/system.h"

namespace real_time
{
// We have different kinds of features we can use for the data in
// Nancy.  However, using a flat structure with an enum makes
// storing and handling it easier than the inheritance based
// approach.  The size overhead is negligible.

enum DataFeatureKind
{
 JustH,
 WithParentH,
};

enum Constants
{
 MAX_SAMPLES = 64,
};

// the struct to collect all possible feature values.  think of it
// like a row in a database.
struct DataFeature
{
  enum DataFeatureKind kind;
  int h;
  int ph;
  DataFeature();
  // constructor to use just h
  DataFeature(int h);
  // constructor to use the parent h too
  DataFeature(int h, int ph);
  // copy constructor
  DataFeature(DataFeature const&);
  // full constructor specifying everything
  DataFeature(DataFeatureKind k, int h, int ph);
  virtual ~DataFeature() = default;

  bool operator==(DataFeature const &other) const;
  int operator-(DataFeature const &o) const;
  size_t hash() const;
};

std::ostream &operator<<(std::ostream &os, DataFeature const &a);
DataFeature read_data_feat(std::stringstream &s, DataFeatureKind k);
DataFeature goal_feature(DataFeatureKind k);


template<typename CountT = int>
struct HStarSample {
  int hstar_value;
  CountT count;
  HStarSample(int a, CountT b) : hstar_value(a), count(b) {}
  ~HStarSample() {}
};

template<typename CountT = int>
struct HStarEntry {
  CountT value_count;
  std::vector<HStarSample<CountT>> hstar_values;
  HStarEntry(int vc) : value_count(vc) {}
  ~HStarEntry() {}
};

template<typename CountT = int>
struct HStarEntries {
  std::vector<DataFeature> features;
  std::vector<HStarEntry<CountT> > values;
  bool empty() const { assert(features.size() == values.size()); return features.size() == 0; };
};

template<typename CountT = int>
struct HStarData {
  std::vector<HStarEntries<CountT> > data;
  HStarData(std::string const &file_name, DataFeatureKind const kind)
  {
    std::ifstream f(file_name);
    std::string line;
    int hs;
    CountT valueCount, hsCount;

    // Note: this could be made a little more efficient with a custom parser.
    while (std::getline(f, line)) {
      std::stringstream ss(line);
      DataFeature const feat = read_data_feat(ss, kind);
      int const h = feat.h;
      ss >> valueCount;

      if (0 == valueCount) {
        // this can happen.  the post expansion belief script sometimes
        // generates empty distributions
        continue;
      }

      data.resize(h+1);
      auto &bin = data[h];

      if (vec_find(bin.features, feat).second) {
        std::cerr << "error: duplicate feat from data " << feat << std::endl;
        utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
      }

      bin.features.emplace_back(feat);
      bin.values.emplace_back(valueCount);
      auto &samples_for_feat = bin.values.back().hstar_values;
      while (!ss.eof()) {
        ss >> hs;
        ss >> hsCount;
        samples_for_feat.emplace_back(hs, hsCount);
      }
    }
    f.close();
  }

  size_t size() const { return data.size(); }
  HStarEntries<CountT> &operator[](size_t idx) { return data[idx]; }
  HStarEntries<CountT> const &operator[](size_t idx) const { return data[idx]; }
};

}

#endif
