#pragma once

#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>

// Frozen WORKED 5% estimator core.
//
// No empirical post-scale and no axis-specific correction are allowed here.
// This is the online form of the estimator frozen in
// docs/WORKED_5_PERCENT_ESTIMATOR.md.
//
// Input correspondences MUST be the already accepted production RANSAC inliers
// from the same adjacent camera interval. The function performs only the frozen
// undistort + 4-parameter similarity fit and converts its translation into a
// metric camera-plane displacement.
namespace worked5 {

constexpr double kFocalScale = 1.10;

struct Step {
  bool valid = false;
  int points = 0;
  double du_norm = 0.0;
  double dv_norm = 0.0;
  double scale = 0.0;
  double yaw = 0.0;
  double dx_m = 0.0;  // frozen blind convention: +dv_norm * height
  double dy_m = 0.0;  // frozen blind convention: -du_norm * height
};

inline Step estimate(const std::vector<cv::Point2f>& prev_inliers,
                     const std::vector<cv::Point2f>& curr_inliers,
                     const cv::Mat& production_K,
                     double production_focal_scale,
                     const cv::Mat& D,
                     double camera_height_m,
                     double dt_s) {
  Step o;
  if (prev_inliers.size() != curr_inliers.size() ||
      prev_inliers.size() < 20 ||
      !(production_focal_scale > 0.0) || !std::isfinite(production_focal_scale) ||
      !(camera_height_m > 0.02) || !std::isfinite(camera_height_m) ||
      !(dt_s > 0.0 && dt_s < 0.2)) {
    return o;
  }

  // Runtime CameraCalib.K has already been multiplied by the production
  // focal_scale. Undo only that scale and apply the frozen WORKED scale. This
  // keeps WORKED at exactly 1.10 even while the AP publisher remains at its
  // independent production focal_scale (historically 0.931).
  cv::Mat K = production_K.clone();
  const double k = kFocalScale / production_focal_scale;
  K.at<double>(0,0) *= k;
  K.at<double>(1,1) *= k;

  std::vector<cv::Point2f> a, b;
  cv::undistortPoints(prev_inliers, a, K, D);
  cv::undistortPoints(curr_inliers, b, K, D);
  if (a.size() != b.size() || a.size() < 20) return o;

  cv::Mat A((int)a.size()*2, 4, CV_64F);
  cv::Mat rhs((int)a.size()*2, 1, CV_64F);
  for (size_t i=0; i<a.size(); ++i) {
    const double x=a[i].x, y=a[i].y;
    const double du=b[i].x-a[i].x;
    const double dv=b[i].y-a[i].y;
    const int r=(int)(2*i);

    A.at<double>(r,0)=1.0; A.at<double>(r,1)=0.0;
    A.at<double>(r,2)=x;   A.at<double>(r,3)=-y;
    rhs.at<double>(r,0)=du;

    A.at<double>(r+1,0)=0.0; A.at<double>(r+1,1)=1.0;
    A.at<double>(r+1,2)=y;   A.at<double>(r+1,3)=x;
    rhs.at<double>(r+1,0)=dv;
  }

  cv::Mat sol;
  if (!cv::solve(A, rhs, sol, cv::DECOMP_SVD) || sol.rows != 4) return o;

  o.points=(int)a.size();
  o.du_norm=sol.at<double>(0,0);
  o.dv_norm=sol.at<double>(1,0);
  o.scale=sol.at<double>(2,0)/dt_s;
  o.yaw=sol.at<double>(3,0)/dt_s;
  o.dx_m=o.dv_norm*camera_height_m;
  o.dy_m=-o.du_norm*camera_height_m;
  o.valid=std::isfinite(o.dx_m) && std::isfinite(o.dy_m) &&
          std::isfinite(o.scale) && std::isfinite(o.yaw);
  return o;
}

} // namespace worked5
