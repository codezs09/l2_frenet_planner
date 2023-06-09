#ifndef UTILS_DEBUG_H
#define UTILS_DEBUG_H

#include <iostream>
#include <string>
#include <vector>

/**
 * @brief Debug functions inside this file should not be used in release.
 *
 */

namespace utils {

inline std::string vector_to_str(const std::vector<double>& vec) {
  std::string str = "[";
  for (std::size_t i = 0; i < vec.size(); ++i) {
    str += std::to_string(vec[i]);
    if (i != vec.size() - 1) {
      str += ", ";
    }
  }
  str += "]";
  return str;
}

inline void print_vector(const std::vector<double>& vec) {
  cout << vector_to_str(vec) << endl;
}

}  // namespace utils

#endif
