#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include "preprocessor.h"
#include "utils.h"

int main() {
  auto cfg = clb::load_cfg(dir_cfg / "config.yaml");
  std::vector<clb::PointCloudT::Ptr> rclds;
  clb::PointCloudT cld;
  clb::load_dir(dir_data / cfg.raw_dirname, rclds);

  YAML::Node node = YAML::LoadFile(dir_cfg / cfg.mat_file_name);
  Eigen::Affine3d T_bias;
  for (size_t i = 0; i < 4; i++) {
    auto row = node["mat"][i];
    for (size_t j = 0; j < 4; j++) {
      T_bias.matrix()(i, j) = row[j].as<double>();
    }
  }

  Eigen::Vector3d axis(cfg.axis_x, cfg.axis_y, cfg.axis_z);
  for (auto rcld : rclds) {
    for (auto& p : rcld->points) {
      Eigen::Vector3d v(p.x, p.y, p.z);
      v = Eigen::Affine3d(Eigen::AngleAxisd(p.rad, axis)) * T_bias * v;
      p.x = v[0];
      p.y = v[1];
      p.z = v[2];
      cld.push_back(p);
    }
  }

  pcl::io::savePCDFile(dir_data / cfg.cmb_filename, cld);

  return 0;
}