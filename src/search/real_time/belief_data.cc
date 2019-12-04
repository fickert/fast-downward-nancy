#include "belief_data.h"

namespace real_time
{

DataFeature::DataFeature(int h) : kind(JustH), h(h) {}
DataFeature::DataFeature(int h, int ph) : kind(WithParentH), h(h), ph(ph) {}
DataFeature::DataFeature(DataFeature const &x) : kind(x.kind), h(x.h), ph(x.ph) {}
DataFeature::DataFeature(DataFeatureKind k, int h, int ph) : kind(k), h(h), ph(ph) {}

DataFeature read_data_feat(std::stringstream &s, DataFeatureKind k)
{
  int a,b;
  switch(k) {
  case JustH:
    s >> a;
    return DataFeature(a);
  case WithParentH:
    s >> a;
    s >> b;
    return DataFeature(a,b);
  }
  assert(false);
#ifdef __GNUC__
  __builtin_unreachable();
#endif
  utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
}

DataFeature goal_feature(DataFeatureKind k)
{
  return DataFeature(k, 0, 1);
}

bool DataFeature::operator==(DataFeature const &other) const
{
  // we shouldn't ever use feature of different kinds in one run
  assert(this->kind == other.kind);
  switch(kind) {
  case JustH:
    return h == other.h;
  case WithParentH:
    return h == other.h && ph == other.ph;
  }
  assert(false);
#ifdef __GNUC__
  __builtin_unreachable();
#endif
  utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
}

int DataFeature::operator-(DataFeature const &o) const
{
  assert(kind == o.kind);
  switch (kind) {
  case real_time::JustH:
    return std::abs(h-o.h);
  case real_time::WithParentH:
    return std::abs(h-o.h) + std::abs(ph-o.ph);
  }
  assert(false);
#ifdef __GNUC__
  __builtin_unreachable();
#endif
  utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
}

std::ostream &operator<<(std::ostream &out, DataFeature const &x)
{
  switch(x.kind) {
  case JustH:
    out << "(0, " << x.h << ")";
    break;
  case WithParentH:
    out << "(1, " << x.h << ", " << x.ph << ")";
    break;
  }
  return out;
}

}
