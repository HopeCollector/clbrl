#include <pcl/io/pcd_io.h>

#include "preprocessor.h"
#include "utils.h"

int main() {
  auto cfg = clb::load_cfg(dir_cfg / "config.yaml");
  std::vector<clb::PointCloudT::Ptr> rclds;
  clb::PointCloudT rcld;
  clb::load_dir(dir_data / cfg.raw_dirname, rclds);
  clb::combine(rclds, rcld, true);
  Eigen::Vector3d axis{cfg.axis_x, cfg.axis_y, cfg.axis_z};
  clb::apply_rot(rcld, rcld, axis);
  pcl::io::savePCDFile(dir_data / cfg.cmb_filename, rcld, true);
  return 0;
}