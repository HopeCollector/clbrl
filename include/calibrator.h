#pragma once
#include <map>

#include "point_type.hpp"

namespace clb {
class Calibrator {
public:
  struct ClbConfig {
    bool is_multi_line = true;
    int num_step = 0;
    double thd_err = 0;
    double thd_chg = 0;
    double thd_gdt = 0;
    bool is_print_progress = true;
    Eigen::Vector3d axis;
  };
  Calibrator() {}
  void init(const ClbConfig&);
  void setInputData(const std::vector<PointCloudT::Ptr>& objs);
  bool run(std::vector<double>& ans);

private:
  struct ThetaFunctor {
  public:
    ThetaFunctor(PointCloudT::Ptr obj, Eigen::Vector3d axis,
                 bool is_multi_line);

    template <typename T>
    bool operator()(const T* const params, T* resdules) const;

  private:
    PointCloudT::ConstPtr obj_;
    std::map<double, PointCloudT::Ptr> planes_, lines_;
    const Eigen::Vector3d axis_;
    bool is_multi_line_ = true;
  };

  struct TransFunctor {
  public:
    TransFunctor(PointCloudT::Ptr obj, Eigen::Matrix3d r_bias,
                 Eigen::Vector3d axis);

    template <typename T>
    bool operator()(const T* const params, T* resdules) const;

  private:
    PointCloudT::ConstPtr obj_;
    const Eigen::Matrix3d r_bias_;
    const Eigen::Vector3d axis_;
  };

  std::vector<PointCloudT::Ptr> objs_;
  ClbConfig cfg_;
};
};  // namespace clb