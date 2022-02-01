#include <pcl-1.10/pcl/io/pcd_io.h>

#include <filesystem>
#include <iostream>
#include <vector>

#include "preprocessor.h"
namespace fs = std::filesystem;

const fs::path HOME(
  "/home/robot/Documents/WorkSpace/lio/test_projs/calibrate/data");

int main() {
  std::vector<clb::PointCloudT::Ptr> rclds;
  clb::load_dir(HOME / "raw_data_2/1", rclds);
  std::cout << rclds.size() << std::endl;

  size_t cnt = 1;
  clb::PointCloudT cld;
  for (auto& rcld : rclds) {
    double r = rcld->at(0).rad;
    for (auto& p : *rcld) {
      if (p.rad - r < 0.02) {
        p.id = cnt;
        cld.push_back(p);
      } else {
        pcl::io::savePCDFile(
          HOME / "raw_data_3" / (std::to_string(cnt) + ".pcd"), cld);
        std::cout << "split: " << cnt << std::endl;
        cnt++;
        cld.clear();
        p.id = cnt;
        cld.push_back(p);
      }

      r = p.rad;
    }
    pcl::io::savePCDFile(HOME / "raw_data_3" / (std::to_string(cnt) + ".pcd"),
                         cld);
    std::cout << "split: " << cnt << std::endl;

    cnt++;
    cld.clear();
  }
  return 0;
}