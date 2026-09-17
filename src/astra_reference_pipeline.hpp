#pragma once

#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstdint>
#include <string>

#include "astra_metric.hpp"
#include "astra_shadow.hpp"

// State wrapper for the literal Astra raw-plane-v1 frontend + metric path.
// This intentionally does NOT reuse metric_odometry_shadow.hpp.
// Semantics mirror estimator/replay.py:
//   - accepted registration advances the tracking anchor;
//   - failed registration keeps the anchor;
//   - SIFT is forced when anchor age > 100 ms;
//   - after a failed attempt with anchor age > 2 s, the anchor is reset;
//   - metric() uses range at the reference/anchor timestamp only.
namespace astra_reference {

struct SensorState {
  astra_metric::M3 body_to_ned=astra_metric::M3::eye();
  double range_m=0.0;
  bool attitude_valid=false;
  bool range_valid=false;
};

struct FrameInput {
  int64_t frame_id=-1;
  int64_t camera_ts_ns=0;
  cv::Mat gray;
  SensorState sensor;
};

enum class Status {
  STARTED=0,
  ACCEPTED,
  IMAGE_REJECT,
  METRIC_REJECT,
  SENSOR_PENDING,
  RESET
};

inline const char* statusName(Status s){
  switch(s){
    case Status::STARTED:return "STARTED";
    case Status::ACCEPTED:return "ACCEPTED";
    case Status::IMAGE_REJECT:return "IMAGE_REJECT";
    case Status::METRIC_REJECT:return "METRIC_REJECT";
    case Status::SENSOR_PENDING:return "SENSOR_PENDING";
    case Status::RESET:return "RESET";
  }
  return "UNKNOWN";
}

struct Step {
  Status status=Status::IMAGE_REJECT;
  int64_t reference_id=-1;
  int64_t current_id=-1;
  int64_t reference_ts_ns=0;
  int64_t current_ts_ns=0;
  double anchor_age_s=0.0;
  bool bridged=false;
  bool reset=false;
  astra_shadow::RegisterInfo registration;
  astra_metric::V3 delta_ned_m{0,0,0};
  astra_metric::V3 position_ned_m{0,0,0};
};

class Pipeline {
 public:
  void clear(){
    started_=false;
    anchor_=FrameInput{};
    position_=astra_metric::V3(0,0,0);
    accepted_=0;
    image_rejected_=0;
    metric_rejected_=0;
    resets_=0;
  }

  bool started() const{return started_;}
  const astra_metric::V3& position() const{return position_;}
  uint64_t accepted() const{return accepted_;}
  uint64_t imageRejected() const{return image_rejected_;}
  uint64_t metricRejected() const{return metric_rejected_;}
  uint64_t resets() const{return resets_;}
  int64_t anchorId() const{return started_?anchor_.frame_id:-1;}

  Step process(const FrameInput& cur){
    Step out;
    out.current_id=cur.frame_id;
    out.current_ts_ns=cur.camera_ts_ns;
    out.position_ned_m=position_;

    if(!started_){
      if(cur.gray.empty()){
        out.status=Status::IMAGE_REJECT;
        return out;
      }
      anchor_=cloneFrame(cur);
      started_=true;
      out.status=Status::STARTED;
      out.reference_id=anchor_.frame_id;
      out.reference_ts_ns=anchor_.camera_ts_ns;
      return out;
    }

    out.reference_id=anchor_.frame_id;
    out.reference_ts_ns=anchor_.camera_ts_ns;
    out.anchor_age_s=std::abs(double(cur.camera_ts_ns-anchor_.camera_ts_ns))*1e-9;
    out.bridged=std::llabs(cur.frame_id-anchor_.frame_id)>1;

    if(cur.gray.empty()){
      ++image_rejected_;
      out.status=Status::IMAGE_REJECT;
      return out;
    }

    // Offline Astra has sensor values for every frame after timeline interpolation.
    // Runtime must keep a frame pending until those values are available; it must
    // not convert temporary absence of the future sample into an image gap.
    if(!anchor_.sensor.attitude_valid || !cur.sensor.attitude_valid ||
       !anchor_.sensor.range_valid){
      out.status=Status::SENSOR_PENDING;
      return out;
    }

    const bool force_sift=out.anchor_age_s>0.100;
    const auto reg=astra_shadow::registerFrames(anchor_.gray,cur.gray,force_sift);
    out.registration=reg.info;

    if(reg.info.ok){
      const auto delta=astra_metric::metric(
          reg.a,reg.b,
          anchor_.sensor.body_to_ned,cur.sensor.body_to_ned,
          anchor_.sensor.range_m);
      if(std::isfinite(delta[0]) && std::isfinite(delta[1]) && std::isfinite(delta[2])){
        position_+=delta;
        ++accepted_;
        out.status=Status::ACCEPTED;
        out.delta_ned_m=delta;
        out.position_ned_m=position_;
        anchor_=cloneFrame(cur);
        return out;
      }
      // estimator/replay.py only advances the anchor when the complete accepted
      // measurement is usable. Keep it here as well so a later frame may bridge.
      ++metric_rejected_;
      out.status=Status::METRIC_REJECT;
      return out;
    }

    ++image_rejected_;
    out.status=Status::IMAGE_REJECT;

    // Literal replay.py reset rule: registration is attempted first; only a
    // failed attempt older than 2 s resets the anchor to the current frame.
    if(out.anchor_age_s>2.0){
      anchor_=cloneFrame(cur);
      ++resets_;
      out.status=Status::RESET;
      out.reset=true;
    }
    return out;
  }

 private:
  static FrameInput cloneFrame(const FrameInput& x){
    FrameInput y=x;
    y.gray=x.gray.clone();
    return y;
  }

  bool started_=false;
  FrameInput anchor_;
  astra_metric::V3 position_{0,0,0};
  uint64_t accepted_=0;
  uint64_t image_rejected_=0;
  uint64_t metric_rejected_=0;
  uint64_t resets_=0;
};

} // namespace astra_reference
