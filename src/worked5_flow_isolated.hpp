#pragma once
#include "runtime.hpp"
#include <array>
struct FeatureRoi { double x0=0.20,y0=0.20,x1=0.80,y1=0.80; };
extern FeatureRoi g_feature_roi;
extern int g_max_features;
extern double g_fb_shadow_max_px;
extern bool g_obs_shadow_enabled;
struct FlowStep {
  bool valid=false;
  int invalid_reason=0; // 0=OK,1=DT,2=FEATURES,3=TRACKED,4=HOMOGRAPHY,5=INLIERS,6=MAGNITUDE
  int features=0,tracked=0,inliers=0;
  bool feature_fallback=false;
  double t_features_ms=0.0,t_lk_ms=0.0,t_ransac_ms=0.0,t_post_ms=0.0;
  double inlier_ratio=0;
  double du_norm=0,dv_norm=0;
  double du_px=0,dv_px=0;
  double yaw_rate_cam_z=0; // fitted optical-axis rotation, rad/s, removed before MAVLink
  double scale_rate=0;      // fitted isotropic image scale rate, 1/s; removed from XY flow
  double lk_height_scale=1; // initial KLT scale guess from TF-Luna, curr image / prev image
  double flow_cam_x=0,flow_cam_y=0;
  double flow_body_x=0,flow_body_y=0;

  // V2 shadow: remove the full known camera rotation ΔR before fitting XY
  // translation/scale. Diagnostic only until A/B tests prove an improvement.
  // Lever-arm shadow: convert optical flow measured at the displaced camera
  // focal point to the FC/IMU reference point using v_cam = omega x r.
  // Proven by repeated bench yaw regression; used for production send when valid.
  bool lever_shadow_valid=false;
  double lever_flow_body_x=0.0,lever_flow_body_y=0.0;
  double lever_pred_flow_x=0.0,lever_pred_flow_y=0.0;
  // Diagnostic A/B shadow path. A is the production result above. B applies
  // forward/backward KLT consistency to the SAME forward correspondences, then
  // runs the same homography RANSAC and 4-parameter fit. B is never sent to FC.
  bool fb_shadow_valid=false;
  int fb_checked=0,fb_pass=0,fb_inliers=0;
  double fb_ratio=0.0;
  double fb_flow_body_x=0.0,fb_flow_body_y=0.0;
  double fb_t_ms=0.0;

  // C shadow: same FB-filtered + RANSAC inliers as B, but the final
  // translation/scale/yaw fit is Huber IRLS instead of ordinary LS.
  // The Huber scale is estimated independently on every frame from MAD of
  // signed 2-D residual components. C is diagnostic only and never published.
  bool robust_shadow_valid=false;
  double robust_flow_body_x=0.0,robust_flow_body_y=0.0;
  double robust_sigma=0.0;
  double robust_mean_weight=0.0;
  int robust_downweighted=0;
  int robust_iters=0;

  // D shadow: same B inliers, but the final 4-parameter fit is weighted by
  // local 2-D observability from the structure-tensor eigenvalue ratio.
  // Weights are normalized to the per-frame median ratio, so there is no
  // absolute brightness/gradient threshold to tune.
  bool obs_shadow_valid=false;
  double obs_flow_body_x=0.0,obs_flow_body_y=0.0;
  double obs_median_ratio=0.0;
  double obs_mean_weight=0.0;
  int obs_downweighted=0;

  std::vector<cv::Point2f> inlier_points; // current-frame RANSAC inliers for web diagnostics
  // Metric-shadow input: preserve BOTH sides of the exact production
  // homography-RANSAC inlier correspondences. Diagnostic only.
  std::vector<cv::Point2f> metric_prev_points;
  std::vector<cv::Point2f> metric_curr_points;

  // 3x3 spatial diagnostics inside the configured feature ROI.
  // Each cell stores median inlier flow transformed to body FRD.
  std::array<int,9> cell_n{};
  std::array<double,9> cell_body_x{};
  std::array<double,9> cell_body_y{};
};
FlowStep estimateRawFlow(const cv::Mat& prev,const cv::Mat& curr,double dt,const CameraCalib& calib,
                         double prev_camera_height_m=0.0,double curr_camera_height_m=0.0,
                         const cv::Matx33d* C1_R_C0=nullptr,double dr_interp_gap_ms=-1.0);
