#pragma once
#include "point_type.hpp"

namespace clb {
// 加载数据, pcd ply
bool load_file(const std::string& file, PointCloudT& cld);

// 从文件夹加载原始数据
bool load_dir(const std::string& dir, std::vector<PointCloudT::Ptr>& clds);

// 将所有原始扫描结果组合起来成为一个完整的环境点云
bool combine(const std::vector<PointCloudT::Ptr>& iclds, PointCloudT& ocld,
             bool is_reidx = false);

// 使用点云中的 rad 旋转点云中的点
void apply_rot(const PointCloudT& icld, PointCloudT& ocld,
               Eigen::Vector3d axis);
void apply_rot(const std::vector<PointCloudT::Ptr>& iclds,
               std::vector<PointCloudT::Ptr>& oclds, Eigen::Vector3d axis);

// 根据提取出来的特征, 将原始数据从 rcld 中提取出来,
// 分别存入不同特征对应的对象中
bool extract(const PointCloudT& rcld, const std::vector<PointCloudT::Ptr>& objs,
             std::vector<PointCloudT::Ptr>& oclds);

// 将 xyzi 转化为 rad id idx
bool toPointCloud(pcl::PCLPointCloud2& rcld, int id, PointCloudT& ocld);
};  // namespace clb