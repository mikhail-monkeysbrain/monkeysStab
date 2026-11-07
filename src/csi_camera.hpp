#pragma once
// CSI YUV420 capture for Pi Zero 2 W. Experimental; no FC or MAVLink writes.
// Buffer is copied before the libcamera request is reused.
#include <libcamera/libcamera.h>
#include <libcamera/formats.h>
#include <libcamera/control_ids.h>
#include <sys/mman.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>
#include <opencv2/core.hpp>

class CsiCamera {
public:
  struct Frame { cv::Mat gray; int64_t sensor_ns=0; uint64_t seq=0; };
  CsiCamera() = default;
  ~CsiCamera() { close(); }
  CsiCamera(const CsiCamera&)=delete;
  CsiCamera& operator=(const CsiCamera&)=delete;

  void open(int fps=30) {
    if(fps<1 || fps>60) throw std::runtime_error("invalid CSI fps");
    if(manager_.start()) throw std::runtime_error("libcamera manager start");
    if(manager_.cameras().empty()) throw std::runtime_error("CSI camera missing");
    camera_=manager_.cameras().front();
    if(camera_->acquire()) throw std::runtime_error("CSI camera acquire");
    config_=camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
    if(!config_ || config_->empty()) throw std::runtime_error("CSI config missing");
    auto& cfg=config_->at(0);
    cfg.size={640,480};
    cfg.pixelFormat=libcamera::formats::YUV420;
    if(config_->validate()==libcamera::CameraConfiguration::Invalid ||
       cfg.size.width!=640 || cfg.size.height!=480 ||
       cfg.pixelFormat!=libcamera::formats::YUV420)
      throw std::runtime_error("CSI 640x480 YUV420 not accepted");
    if(camera_->configure(config_.get())) throw std::runtime_error("CSI configure");
    stride_=cfg.stride;
    allocator_=std::make_unique<libcamera::FrameBufferAllocator>(camera_);
    auto *stream=cfg.stream();
    if(allocator_->allocate(stream)<0) throw std::runtime_error("CSI allocate");
    for(const auto& b:allocator_->buffers(stream)){
      const auto& planes=b->planes();
      if(planes.empty()) throw std::runtime_error("CSI no planes");
      // YUV420 may use one dmabuf for all planes. Only first Y plane is needed.
      const auto& p=planes[0];
      const long page=sysconf(_SC_PAGESIZE);
      const off_t offset=static_cast<off_t>(p.offset);
      const off_t aligned=offset & ~(static_cast<off_t>(page)-1);
      const size_t delta=static_cast<size_t>(offset-aligned);
      const size_t len=static_cast<size_t>(p.length)+delta;
      void* base=mmap(nullptr,len,PROT_READ,MAP_SHARED,p.fd.get(),aligned);
      if(base==MAP_FAILED) throw std::runtime_error("CSI mmap");
      maps_.push_back({base,len,static_cast<const uint8_t*>(base)+delta});
      auto req=camera_->createRequest();
      if(!req || req->addBuffer(stream,b.get())) throw std::runtime_error("CSI request");
      requests_.push_back(std::move(req));
    }
    camera_->requestCompleted.connect(this,&CsiCamera::completed);
    libcamera::ControlList controls(camera_->controls());
    const int64_t us=1000000/fps;
    const std::array<int64_t,2> durations{us,us};
    controls.set(libcamera::controls::FrameDurationLimits,
                 std::span<const int64_t,2>(durations));
    if(camera_->start(&controls)) throw std::runtime_error("CSI start");
    running_=true;
    for(auto& req:requests_)
      if(camera_->queueRequest(req.get())) throw std::runtime_error("CSI queue");
  }

  // Latest-only delivery. Caller owns cv::Mat memory after return.
  bool read(Frame& out,int timeout_ms=100) {
    std::unique_lock<std::mutex> lk(mu_);
    if(!cv_.wait_for(lk,std::chrono::milliseconds(timeout_ms),
                     [&]{return ready_ || !running_;})) return false;
    if(!ready_) return false;
    out.gray=std::move(latest_.gray);
    out.sensor_ns=latest_.sensor_ns;
    out.seq=latest_.seq;
    ready_=false;
    return true;
  }

  void close() {
    if(camera_) {
      { std::lock_guard<std::mutex> lk(mu_); running_=false; }
      cv_.notify_all();
      camera_->stop();
      camera_->requestCompleted.disconnect(this);
      requests_.clear();
      for(auto& m:maps_) if(m.base && m.base!=MAP_FAILED) munmap(m.base,m.len);
      maps_.clear();
      allocator_.reset();
      config_.reset();
      camera_->release();
      camera_.reset();
    }
    manager_.stop();
  }
private:
  struct Mapping { void* base; size_t len; const uint8_t* pixels; };
  void completed(libcamera::Request* req) {
    if(req->status()==libcamera::Request::RequestCancelled) return;
    auto* fb=req->buffers().begin()->second;
    size_t index=0;
    for(;index<requests_.size();++index)
      if(requests_[index].get()==req) break;
    if(index>=maps_.size()) return;
    const auto timestamp=req->metadata().get(libcamera::controls::SensorTimestamp);
    if(timestamp && fb->metadata().status==libcamera::FrameMetadata::FrameSuccess) {
      cv::Mat gray(480,640,CV_8UC1);
      const uint8_t* src=maps_[index].pixels;
      for(int y=0;y<480;++y)
        std::memcpy(gray.ptr(y),src+static_cast<size_t>(y)*stride_,640);
      {
        std::lock_guard<std::mutex> lk(mu_);
        latest_.gray=std::move(gray);
        latest_.sensor_ns=*timestamp;
        latest_.seq=++seq_;
        ready_=true;
      }
      cv_.notify_one();
    }
    req->reuse(libcamera::Request::ReuseBuffers);
    bool active;
    { std::lock_guard<std::mutex> lk(mu_); active=running_; }
    if(active) camera_->queueRequest(req);
  }
  libcamera::CameraManager manager_;
  std::shared_ptr<libcamera::Camera> camera_;
  std::unique_ptr<libcamera::CameraConfiguration> config_;
  std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
  std::vector<std::unique_ptr<libcamera::Request>> requests_;
  std::vector<Mapping> maps_;
  unsigned stride_=0;
  std::mutex mu_;
  std::condition_variable cv_;
  Frame latest_;
  uint64_t seq_=0;
  bool ready_=false,running_=false;
};
