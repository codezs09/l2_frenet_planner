#include "utils.h"

bool LoadJsonFile(string scene_path, json* j) {
  ifstream f(scene_path);
  if (!f.is_open()) {
    cout << "Failed to open file: " << scene_path << endl;
    return false;
  }

  std::string s((std::istreambuf_iterator<char>(f)),
                std::istreambuf_iterator<char>());

  if (s.empty()) {
    cout << "File is empty: " << scene_path << endl;
    return false;
  }

  try {
    *j = json::parse(s);
  } catch (const std::exception& e) {
    cout << "Json parsing error for file: " << scene_path << endl;
    return false;
  }

  f.close();
  return true;
}
