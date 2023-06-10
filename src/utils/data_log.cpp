#include "data_log.h"

bool save_data(const std::string& filename,
               const std::vector<DataFrame>& data_frames) {
  std::ofstream ofs(filename, std::ios::binary);
  if (!ofs.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return false;
  }
  msgpack::pack(ofs, data_frames);
  ofs.close();
  return true;
}
