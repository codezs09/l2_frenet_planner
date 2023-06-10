#ifndef LOOKUP_TABLE_1D_H
#define LOOKUP_TABLE_1D_H

#include <algorithm>
#include <cassert>
#include <map>

using namespace std;

namespace utils {

class LookupTable1D {
 private:
  std::map<double, double> table;

 public:
  LookupTable1D() = delete;
  LookupTable1D(const std::map<double, double>& table_) : table(table_) {
    assert(table.size() > 0);
  }

  // copy constructor
  LookupTable1D(const LookupTable1D& other) : table(other.table) {}

  // copy assignment operator, copy and swap idiom
  LookupTable1D& operator=(LookupTable1D other) {
    if (this != &other) {
      LookupTable1D copy(other);
      std::swap(*this, copy);
    }
    return *this;
  }

  double operator[](double x) const { return get(x); }

  double get(double x) const {
    // check for exact match
    auto it = table.find(x);
    if (it != table.end()) {
      return it->second;
    }

    // check for extrapolation cases
    if (x < table.begin()->first) {
      return table.begin()->second;
    }
    auto it_upper = table.upper_bound(x);
    if (it_upper == table.end()) {
      return table.rbegin()->second;
    }

    // linear interpolation
    auto it_lower = std::prev(it_upper);
    double t = (x - it_lower->first) / (it_upper->first - it_lower->first);
    return (1 - t) * it_lower->second + t * it_upper->second;
  }
};

}  // namespace utils

#endif
