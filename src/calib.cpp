#include "calibrator.h"
#include "cfg.h"
#include "preprocessor.h"
#include "utils.h"

int main() {
  auto cfg = clb::load_cfg(dir_cfg / "config.yaml");
  std::vector<clb::PointCloudT::Ptr> rclds;
  clb::PointCloudT rcld;
  clb::load_dir(dir_data / cfg.raw_dirname, rclds);
  clb::combine(rclds, rcld, true);
  Eigen::Vector3d axis{cfg.axis_x, cfg.axis_y, cfg.axis_z};

  std::vector<clb::PointCloudT::Ptr> objs;
  clb::load_dir(dir_data / cfg.obj_dirname, objs);
  clb::extract(rcld, objs, objs);

  clb::Calibrator cr;
  clb::Calibrator::ClbConfig ccfg;
  ccfg.is_multi_line = cfg.is_multi_line;
  ccfg.is_print_progress = cfg.is_print_progress;
  ccfg.num_step = cfg.num_step;
  ccfg.thd_chg = cfg.thd_chg;
  ccfg.thd_err = cfg.thd_err;
  ccfg.thd_gdt = cfg.thd_gdt;
  ccfg.axis = axis;
  cr.init(ccfg);
  cr.setInputData(objs);

  std::vector<double> ans;
  cr.run(ans);
  clb::save_ans(dir_cfg / cfg.mat_file_name, ans);

  for (auto& d : ans) {
    std::cout << d << " ";
  }
  std::cout << std::endl;

  return 0;
}