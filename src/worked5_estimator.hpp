#pragma once

#include <opencv2/opencv.hpp>
#include <algorithm>
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

inline bool normalizedCorrespondences(
    const std::vector<cv::Point2f>& prev_inliers,
    const std::vector<cv::Point2f>& curr_inliers,
    const cv::Mat& production_K,
    double production_focal_scale,
    const cv::Mat& D,
    std::vector<cv::Point2f>* prev_norm,
    std::vector<cv::Point2f>* curr_norm) {
  if (!prev_norm || !curr_norm ||
      prev_inliers.size() != curr_inliers.size() ||
      prev_inliers.size() < 20 ||
      !(production_focal_scale > 0.0) || !std::isfinite(production_focal_scale)) {
    return false;
  }

  cv::Mat K = production_K.clone();
  const double k = kFocalScale / production_focal_scale;
  K.at<double>(0,0) *= k;
  K.at<double>(1,1) *= k;

  cv::undistortPoints(prev_inliers, *prev_norm, K, D);
  cv::undistortPoints(curr_inliers, *curr_norm, K, D);
  return prev_norm->size() == curr_norm->size() && prev_norm->size() >= 20;
}

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

  std::vector<cv::Point2f> a, b;
  if (!normalizedCorrespondences(prev_inliers, curr_inliers,
                                 production_K, production_focal_scale, D,
                                 &a, &b)) return o;

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

// Diagnostic-only nested model for the WORKED5 causal experiment.
// It uses the exact same accepted production RANSAC correspondences and the
// exact same WORKED5 normalization, but fixes scale=yaw=0.  It never feeds FC.
struct TranslationShadow {
  bool valid=false;
  int points=0;
  double tls_du_norm=0.0,tls_dv_norm=0.0;
  double tmed_du_norm=0.0,tmed_dv_norm=0.0;
  double tls_dx_m=0.0,tls_dy_m=0.0;
  double tmed_dx_m=0.0,tmed_dy_m=0.0;
};

inline double median(std::vector<double> v) {
  if (v.empty()) return 0.0;
  const size_t m=v.size()/2;
  std::nth_element(v.begin(),v.begin()+m,v.end());
  const double hi=v[m];
  if (v.size()&1U) return hi;
  std::nth_element(v.begin(),v.begin()+m-1,v.begin()+m);
  return 0.5*(v[m-1]+hi);
}

inline TranslationShadow estimateTranslationOnly(
    const std::vector<cv::Point2f>& prev_inliers,
    const std::vector<cv::Point2f>& curr_inliers,
    const cv::Mat& production_K,
    double production_focal_scale,
    const cv::Mat& D,
    double camera_height_m,
    double dt_s) {
  TranslationShadow o;
  if (!(camera_height_m>0.02) || !std::isfinite(camera_height_m) ||
      !(dt_s>0.0 && dt_s<0.2)) return o;

  std::vector<cv::Point2f> a,b;
  if (!normalizedCorrespondences(prev_inliers,curr_inliers,
                                 production_K,production_focal_scale,D,
                                 &a,&b)) return o;

  std::vector<double> dus,dvs;
  dus.reserve(a.size()); dvs.reserve(a.size());
  double sum_du=0.0,sum_dv=0.0;
  for(size_t i=0;i<a.size();++i){
    const double du=(double)b[i].x-a[i].x;
    const double dv=(double)b[i].y-a[i].y;
    if(!std::isfinite(du)||!std::isfinite(dv)) return o;
    dus.push_back(du); dvs.push_back(dv);
    sum_du+=du; sum_dv+=dv;
  }
  o.points=(int)dus.size();
  o.tls_du_norm=sum_du/dus.size();
  o.tls_dv_norm=sum_dv/dvs.size();
  o.tmed_du_norm=median(dus);
  o.tmed_dv_norm=median(dvs);
  o.tls_dx_m=o.tls_dv_norm*camera_height_m;
  o.tls_dy_m=-o.tls_du_norm*camera_height_m;
  o.tmed_dx_m=o.tmed_dv_norm*camera_height_m;
  o.tmed_dy_m=-o.tmed_du_norm*camera_height_m;
  o.valid=std::isfinite(o.tls_dx_m)&&std::isfinite(o.tls_dy_m)&&
          std::isfinite(o.tmed_dx_m)&&std::isfinite(o.tmed_dy_m);
  return o;
}

} // namespace worked5
