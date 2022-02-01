#pragma once
#include <filesystem>
#include <string>
#include <vector>

#include "cfg.h"

const std::filesystem::path dir_home(DIR_HOME);

namespace clb {
std::filesystem::path choose_file(const std::string &dir,
                                  const std::string &prefix = "");

int get_id(const std::string &filename);

Config load_cfg();

bool save_ans(const std::string &filename, const std::vector<double> &ans);
};  // namespace clb