#include <pcl-1.10/pcl/io/pcd_io.h>

#include <filesystem>
#include <iostream>
#include <vector>

#include "preprocessor.h"
namespace fs = std::filesystem;

const fs::path HOME(
  "/home/robot/Documents/WorkSpace/lio/test_projs/clbrl/data");

int main() {
  std::vector<clb::PointCloudT::Ptr> rclds;
  clb::load_dir(HOME / "rb3", rclds);
  std::cout << rclds[100]->size() << std::endl;

  return 0;
}