#pragma once
#define PCL_NO_PRECOMPILE

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
struct PointXYZRadID {
  PCL_ADD_POINT4D;
  float rad;
  float id;
  float idx;
  PCL_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN64;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRadID,     //
                                  (float, x, x)      //
                                  (float, y, y)      //
                                  (float, z, z)      //
                                  (float, rad, rad)  //
                                  (float, id, id)    //
                                  (float, idx, idx)  //
)

namespace clb {
using PointT = PointXYZRadID;
using PointCloudT = pcl::PointCloud<PointT>;

using RawPointT = pcl::PointXYZI;
using RawPointCloudT = pcl::PointCloud<RawPointT>;
}  // namespace clb