#include "belief_data.h"

namespace real_time
{

DataFeature::DataFeature(int h) : kind(JustH), h(h) {}
DataFeature::DataFeature(int h, int ph) : kind(WithParentH), h(h), ph(ph) {}
DataFeature::DataFeature(DataFeature const &x) : kind(x.kind), h(x.h), ph(x.ph) {}

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
  __builtin_unreachable();
}

DataFeature goal_feature(DataFeatureKind k)
{
  switch(k) {
  case JustH:
    return DataFeature(0);
  case WithParentH:
    return DataFeature(0,1);
  }
  assert(false);
  __builtin_unreachable();
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
  __builtin_unreachable();
}

bool DataFeature::weak_eq(DataFeature const &other) const
{
  return h == other.h;
}

size_t DataFeature::hash() const
{
  switch (kind) {
  case real_time::JustH:
    return h;
  case real_time::WithParentH:
    return (h ^ 17) << 16 & ph;
  }
  assert(false);
  __builtin_unreachable();
}

void DataFeature::dec_h()
{
  --h;
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
  __builtin_unreachable();
}

std::ostream &operator<<(std::ostream &out, DataFeature const &x)
{
  switch(x.kind) {
  case JustH:
    out << x.h;
    break;
  case WithParentH:
    out << x.h << ", " << x.ph;
    break;
  }
  return out;
}

}