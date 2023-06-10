#include "data_log.h"

void save_data(const std::string& filename,
               const std::vector<DataFrame>& data_frames) {
  std::ofstream ofs(filename, std::ios::binary);
  msgpack::pack(ofs, data_frames);
  ofs.close();
}
