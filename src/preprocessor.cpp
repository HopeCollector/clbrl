#include "preprocessor.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Eigen>
#include <filesystem>
#include <regex>

namespace fs = std::filesystem;
using namespace clb;

bool clb::load_file(const std::string& file, PointCloudT& cld) {
  fs::path p(file);
  auto file_type = p.extension().string();
  auto file_name = p.filename().string();

  // get id
  static const std::regex pattern("[0-9]+");
  std::smatch ans;
  std::regex_search(file_name, ans, pattern);
  int id = ans.size() == 1 ? std::stoi(ans[0]) : -1;

  pcl::PCLPointCloud2 rcld;
  if (file_type == ".pcd") {
    pcl::io::loadPCDFile(p, rcld);
  } else if (file_type == ".ply") {
    pcl::io::loadPLYFile(p, rcld);
  } else {
    std::cerr << "Cannot load type: " << file_type << std::endl;
    return false;
  }

  return toPointCloud(rcld, id, cld);
}

bool clb::load_dir(const std::string& dir,
                   std::vector<PointCloudT::Ptr>& clds) {
  if (!fs::is_directory(dir)) {
    std::cerr << dir << " isn't a directory" << std::endl;
    return false;
  }

  for (auto& entry : fs::directory_iterator(dir)) {
    if (entry.is_directory()) {
      load_dir(entry.path(), clds);
    } else {
      PointCloudT::Ptr pcld(new PointCloudT);
      if (load_file(entry.path(), *pcld))
        clds.push_back(pcld);
      else
        return false;
    }
  }

  return true;
}

bool clb::combine(const std::vector<PointCloudT::Ptr>& iclds, PointCloudT& ocld,
                  bool is_reidx) {
  ocld.clear();
  ocld.reserve(iclds.size() * iclds[0]->size());
  for (size_t i = 0; i < iclds.size(); i++) {
    ocld += *iclds[i];
  }
  if (is_reidx) {
    for (size_t i = 0; i < ocld.size(); i++) {
      ocld[i].idx = i;
    }
  }

  return true;
}

void clb::apply_rot(const PointCloudT& icld, PointCloudT& ocld,
                    Eigen::Vector3d axis) {
  axis.normalize();
  PointCloudT tmp;
  tmp.resize(icld.size());

  for (size_t i = 0; i < icld.size(); i++) {
    tmp[i] = icld[i];
    Eigen::Vector3d vec(icld[i].x, icld[i].y, icld[i].z);
    vec = Eigen::AngleAxisd(icld[i].rad, axis) * vec;
    tmp[i].x = vec[0];
    tmp[i].y = vec[1];
    tmp[i].z = vec[2];
  }

  ocld.swap(tmp);
}

void clb::apply_rot(const std::vector<PointCloudT::Ptr>& iclds,
                    std::vector<PointCloudT::Ptr>& oclds,
                    Eigen::Vector3d axis) {
  std::vector<PointCloudT::Ptr> tmp;
  tmp.resize(iclds.size());
  for (size_t i = 0; i < iclds.size(); i++) {
    tmp[i].reset(new PointCloudT);
    apply_rot(*iclds[i], *tmp[i], axis);
  }
  oclds.swap(tmp);
}

bool clb::extract(const PointCloudT& rcld,
                  const std::vector<PointCloudT::Ptr>& objs,
                  std::vector<PointCloudT::Ptr>& oclds) {
  std::vector<PointCloudT::Ptr> tmp;
  tmp.resize(objs.size());
  for (size_t i = 0; i < tmp.size(); i++) {
    tmp[i].reset(new PointCloudT);
    tmp[i]->reserve(objs[i]->size());
  }

  for (size_t i = 0; i < objs.size(); i++) {
    const auto& obj = objs[i];
    auto& ocld = tmp[i];
    for (const auto& p : obj->points) {
      ocld->push_back(rcld[p.idx]);
      ocld->back().id = p.id;
    }
  }

  oclds.swap(tmp);
  return true;
}

bool clb::toPointCloud(const pcl::PCLPointCloud2& rcld, int id,
                       PointCloudT& ocld) {
  bool is_has_intensity = false;
  bool is_has_rad = false;
  for (const auto& field : rcld.fields) {
    if (field.name == "intensity") {
      is_has_intensity = true;
    } else if (field.name == "rad") {
      is_has_rad = true;
    }
    if (is_has_intensity || is_has_rad) {
      break;
    }
  }

  if (!(is_has_intensity || is_has_rad)) {
    std::cerr << "Cannot convert these fileds: ";
    for (const auto& field : rcld.fields) {
      std::cerr << field.name << " ";
    }
    std::cerr << std::endl;
    return false;
  }

  if (is_has_intensity) {
    RawPointCloudT icld;
    pcl::fromPCLPointCloud2(rcld, icld);
    ocld.reserve(icld.size());
    PointT po;
    for (size_t i = 0; i < icld.size(); i++) {
      const auto& pi = icld[i];
      po.x = pi.x;
      po.y = pi.y;
      po.z = pi.z;
      po.rad = pi.intensity * M_PI / 180.0;
      po.id = id;
      po.idx = i;
      ocld.push_back(po);
    }
  } else if (is_has_rad) {
    pcl::fromPCLPointCloud2(rcld, ocld);
  }

  return true;
}