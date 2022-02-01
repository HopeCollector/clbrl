#pragma once
#include <string>
namespace clb {
struct Config {
  std::string raw_dirname;
  std::string cmb_filename;
  std::string obj_dirname;
  std::string mat_file_name;
  bool is_multi_line = true;
  int num_step = 0;
  double thd_err = 0;
  double thd_chg = 0;
  double thd_gdt = 0;
  bool is_print_progress = true;
  double axis_x = 0;
  double axis_y = 0;
  double axis_z = 0;
};

};  // namespace clb