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

template <typename T>
inline std::string vector_to_str(const std::vector<T>& vec) {
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

template <typename T>
inline void print_vector(const std::vector<T>& vec) {
  std::cout << vector_to_str(vec) << std::endl;
}

}  // namespace utils

#endif
