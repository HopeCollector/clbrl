#include "utils.h"

#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <regex>

#include "cfg.h"
#include "yaml-cpp/yaml.h"

namespace fs = std::filesystem;

fs::path clb::choose_file(const std::string &dir, const std::string &prefix) {
  std::vector<std::string> dirs;
  for (auto &entry : fs::directory_iterator(dir)) {
    if (!prefix.empty() && entry.path().filename().string().find(prefix) == 0) {
      dirs.push_back(entry.path().string());
    } else if (prefix.empty() && entry.is_directory()) {
      dirs.push_back(entry.path().string());
    }
  }

  std::sort(dirs.begin(), dirs.end());
  for (int i = 0; i < dirs.size(); i++) {
    std::cout << "[" << i << "]: " << dirs[i] << std::endl;
  }
  std::cout << "which to use ? (0 ~ [" << dirs.size() - 1 << "]): ";
  std::string tmp;
  std::getline(std::cin, tmp);
  if (!tmp.empty()) {
    tmp = dirs[std::stoi(tmp)];
  } else {
    tmp = dirs.back();
  }
  std::cout << "You choose: " << tmp << std::endl;
  // std::cin.ignore(std::numeric_limits<std::streamsize>::max()); // clear cin
  std::cin.clear();
  return {tmp};
}

int clb::get_id(const std::string &filename) {
  static const std::regex pattern("[0-9]+");
  std::smatch ans;
  std::regex_search(filename, ans, pattern);
  return ans.size() > 0 ? std::stoi(ans[0]) : -1;
}

clb::Config clb::load_cfg(const std::string &cfg_name) {
  YAML::Node node = YAML::LoadFile(cfg_name);
  clb::Config cfg;
  cfg.raw_dirname = node["raw_dirname"].as<std::string>();
  cfg.cmb_filename = node["cmb_filename"].as<std::string>();
  cfg.obj_dirname = node["obj_dirname"].as<std::string>();
  cfg.mat_file_name = node["mat_file_name"].as<std::string>();
  cfg.is_multi_line = node["is_multi_line"].as<bool>();
  cfg.num_step = node["num_step"].as<int>();
  cfg.thd_err = node["thd_err"].as<double>();
  cfg.thd_chg = node["thd_chg"].as<double>();
  cfg.thd_gdt = node["thd_gdt"].as<double>();
  cfg.is_print_progress = node["is_print_progress"].as<bool>();
  cfg.axis_x = node["axis"][0].as<double>();
  cfg.axis_y = node["axis"][1].as<double>();
  cfg.axis_z = node["axis"][2].as<double>();
  return cfg;
}

bool clb::save_ans(const std::string &filename,
                   const std::vector<double> &ans) {
  YAML::Node node;
  node["theta"].push_back(ans[0]);
  node["theta"].push_back(ans[1]);
  node["theta"].push_back(ans[2]);
  node["trans"].push_back(ans[3]);
  node["trans"].push_back(ans[4]);
  node["trans"].push_back(ans[5]);

  // Eigen::Affine3d T_bias;
  auto T_bias = Eigen::Affine3d()
                  .fromPositionOrientationScale(
                    Eigen::Vector3d{ans[3], ans[4], ans[5]},
                    Eigen::AngleAxisd(ans[0], Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(ans[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(ans[2], Eigen::Vector3d::UnitZ()),
                    Eigen::Vector3d{1, 1, 1})
                  .matrix();

  for (size_t i = 0; i < 4; i++) {
    YAML::Node row;
    for (size_t j = 0; j < 4; j++) {
      row.push_back(T_bias(i, j));
    }
    node["mat"].push_back(row);
  }

  std::ofstream file(filename);
  if (file.is_open()) {
    YAML::Emitter outer;
    outer << node;
    file << outer.c_str();
  } else {
    std::cerr << "Cannot open file: " << filename << std::endl;
    return false;
  }

  return true;
}