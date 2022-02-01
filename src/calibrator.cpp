#include "calibrator.h"

#include <ceres/ceres.h>

using namespace clb;

// 在施加 bias 的前提下将所有原始点组合成一个 li 坐标系下的平面
// r 为旋转偏移量
template <typename T>
static Eigen::Matrix<T, 3, -1> combine_points(const PointCloudT::ConstPtr obj,
                                              const Eigen::Matrix<T, 3, 3>& r,
                                              const Eigen::Vector3d& axis) {
  Eigen::Matrix<T, -1, -1> points(3, obj->size());

  Eigen::Vector3d vec;
  for (size_t i = 0; i < obj->size(); i++) {
    const auto& p = obj->at(i);
    vec << p.x, p.y, p.z;
    points.col(i) =
      Eigen::AngleAxisd(p.rad, axis).cast<T>() * r * vec.cast<T>();
  }

  return points;
}

template <typename T>
static Eigen::Matrix<T, 3, -1> combine_points(
  const PointCloudT::ConstPtr obj,
  const Eigen::Transform<T, 3, Eigen::Affine>& T_bias,
  const Eigen::Vector3d& axis) {
  Eigen::Matrix<T, -1, -1> points(3, obj->size());

  Eigen::Vector3d vec;
  for (size_t i = 0; i < obj->size(); i++) {
    const auto& p = obj->at(i);
    vec << p.x, p.y, p.z;
    points.col(i) = Eigen::Transform<T, 3, Eigen::Affine>(
                      Eigen::AngleAxis<T>(T(p.rad), axis.cast<T>())) *
                    T_bias * vec.cast<T>();
  }

  return points;
}

template <typename T>
static Eigen::Matrix<T, 3, 1> eigenvector(Eigen::Matrix<T, 3, 3> mat) {
  const static double tolerance = 1e-10;
  const static size_t maxiternum = 500;
  Eigen::Matrix<T, 3, 1> x = Eigen::Vector3d::Random().cast<T>();
  Eigen::Matrix<T, 3, 1> y_0;
  Eigen::Matrix<T, 3, 1> y_1 = x / x.maxCoeff();
  int cnt = 0;

  do {
    y_0 = y_1;
    x = mat * y_0;
    y_1 = x / x.maxCoeff();
    cnt++;
  } while ((y_1 - y_0).norm() > tolerance && cnt < maxiternum);

  y_1.normalize();
  return y_1;
}

template <typename T>
static Eigen::Matrix<T, 3, 1> inveigenvector(Eigen::Matrix<T, 3, 3> mat) {
  return eigenvector(Eigen::Matrix<T, 3, 3>(mat.inverse()));
}

void Calibrator::init(const Calibrator::ClbConfig& cfg) { cfg_ = cfg; }

void Calibrator::setInputData(const std::vector<PointCloudT::Ptr>& objs) {
  objs_ = objs;
}

bool Calibrator::run(std::vector<double>& ans) {
  if (cfg_.num_step == 0) {
    std::cerr << "Calibrator Config Error: Need call init first!" << std::endl;
    return false;
  }

  // 配置优化选项与优化变量
  ceres::Solver::Options options;
  double theta[3]{0};
  double trans[3]{0};
  {
    options.max_num_iterations = cfg_.num_step;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = cfg_.is_print_progress;
    options.function_tolerance = cfg_.thd_err;
    options.gradient_tolerance = cfg_.thd_gdt;
    options.parameter_tolerance = cfg_.thd_chg;
  }

  // optimize rot
  {
    std::cout << "Optmize Rotation!!!" << std::endl;
    ceres::Problem pbm;
    for (auto& obj : objs_) {
      ceres::CostFunction* cf =
        new ceres::AutoDiffCostFunction<ThetaFunctor, 1, 3>(
          new ThetaFunctor(obj, cfg_.axis, cfg_.is_multi_line));
      pbm.AddResidualBlock(cf, nullptr, &theta[0]);
    }

    ceres::Solver::Summary smy;
    ceres::Solve(options, &pbm, &smy);
    std::cout << smy.BriefReport() << std::endl;
  }

  // optmize trans
  {
    std::cout << "Optmize Translation!!!!" << std::endl;
    ceres::Problem pbm;
    Eigen::Matrix3d r_bias =
      Eigen::AngleAxisd(theta[0], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(theta[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(theta[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();

    for (auto& obj : objs_) {
      ceres::CostFunction* cf =
        new ceres::AutoDiffCostFunction<TransFunctor, 1, 3>(
          new TransFunctor(obj, r_bias, cfg_.axis));
      pbm.AddResidualBlock(cf, nullptr, &trans[0]);
    }

    ceres::Solver::Summary smy;
    ceres::Solve(options, &pbm, &smy);
    std::cout << smy.BriefReport() << std::endl;
  }

  ans.resize(6);
  ans[0] = theta[0];
  ans[1] = theta[1];
  ans[2] = theta[2];
  ans[3] = trans[0];
  ans[4] = trans[1];
  ans[5] = trans[2];

  return true;
}

Calibrator::ThetaFunctor::ThetaFunctor(PointCloudT::Ptr obj,
                                       Eigen::Vector3d axis, bool is_multi_line)
    : obj_(obj), axis_(axis), is_multi_line_(is_multi_line) {
  if (is_multi_line) {
    for (size_t i = 0; i < obj->size(); i++) {
      const auto& p = obj->at(i);
      if (planes_.count(p.id)) {
        planes_[p.id]->push_back(p);
      } else {
        planes_[p.id] = pcl::make_shared<PointCloudT>();
        planes_[p.id]->push_back(p);
      }
    }

    auto iter = planes_.begin();
    while (iter != planes_.end()) {
      if (iter->second->size() < 20) {
        iter = planes_.erase(iter);
      } else {
        iter++;
      }
    }
  } else {
    for (size_t i = 0; i < obj->size(); i++) {
      const auto& p = obj->at(i);
      if (lines_.count(p.id)) {
        lines_[p.id]->push_back(p);
      } else {
        lines_[p.id] = pcl::make_shared<PointCloudT>();
        lines_[p.id]->push_back(p);
      }
    }
  }
}

template <typename T>
bool Calibrator::ThetaFunctor::operator()(const T* const params,
                                          T* resdules) const {
  Eigen::Matrix<T, 3, 3> r_lc_lcb(
    Eigen::AngleAxis<T>(params[0], Eigen::Vector3d::UnitX().cast<T>()) *
    Eigen::AngleAxis<T>(params[1], Eigen::Vector3d::UnitY().cast<T>()) *
    Eigen::AngleAxis<T>(params[2], Eigen::Vector3d::UnitZ().cast<T>()));

  Eigen::Matrix<T, 3, -1> plane = combine_points(obj_, r_lc_lcb, axis_);
  plane = plane.colwise() - plane.rowwise().sum() / T(plane.cols());
  Eigen::Matrix<T, 3, 1> plane_normal =
    inveigenvector(Eigen::Matrix<T, 3, 3>(plane * plane.transpose()));

  resdules[0] = T{0};
  if (is_multi_line_) {
    // 多线激光扫描的结果是面, 应该使用 面面(n, n) 法向量之间的误差
    for (const auto& [id, pc] : planes_) {
      Eigen::Matrix<T, 3, -1> subpln = combine_points(pc, r_lc_lcb, axis_);
      subpln = subpln.colwise() - subpln.rowwise().sum() / T(subpln.cols());
      Eigen::Matrix<T, 3, 1> subn =
        inveigenvector(Eigen::Matrix<T, 3, 3>(subpln * subpln.transpose()));
      T err = T(subn.transpose() * plane_normal);  // 1 - n1 * n2
      if (err < T(0)) {
        err = T(1) + err;
      } else {
        err = T(1) - err;
      }
      resdules[0] += err * err;
    }
  } else {
    // 单线激光扫描的结果是线, 应该使用 线面(l, n) 法向量之间的误差
    for (const auto& [id, pc] : lines_) {
      Eigen::Matrix<T, 3, -1> subl = combine_points(pc, r_lc_lcb, axis_);
      subl = subl.colwise() - subl.rowwise().sum() / T(subl.cols());
      Eigen::Matrix<T, 3, 1> subn =
        eigenvector(Eigen::Matrix<T, 3, 3>(subl * subl.transpose()));
      T err = subn.transpose() * plane_normal;  // l * n
      resdules[0] += err * err;
    }
  }
  resdules[0] = ceres::sqrt(resdules[0]) / T(planes_.size());
  return true;
}

Calibrator::TransFunctor::TransFunctor(PointCloudT::Ptr obj,
                                       Eigen::Matrix3d r_bias,
                                       Eigen::Vector3d axis)
    : obj_(obj), r_bias_(r_bias), axis_(axis) {}

template <typename T>
bool Calibrator::TransFunctor::operator()(const T* const params,
                                          T* resdules) const {
  Eigen::Transform<T, 3, Eigen::Affine> T_bias(
    Eigen::Matrix4d::Identity().cast<T>());
  T_bias.prerotate(r_bias_.cast<T>())
    .pretranslate(Eigen::Matrix<T, 3, 1>(params[0], params[1], params[1]));

  Eigen::Matrix<T, 3, -1> plane = combine_points(obj_, T_bias, axis_);
  Eigen::Matrix<T, 3, -1> plane_centered =
    plane.colwise() - plane.rowwise().sum() / T(plane.cols());
  Eigen::Matrix<T, 3, 1> plane_normal = inveigenvector(
    Eigen::Matrix<T, 3, 3>(plane_centered * plane_centered.transpose()));
  Eigen::Matrix<T, 1, -1> distances = plane_normal.transpose() * plane;
  T mean_dis = distances.sum() / T(distances.cols());
  resdules[0] =
    (distances.array() - mean_dis).matrix().squaredNorm() / T(distances.cols());

  return true;
}