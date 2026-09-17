#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace astra_shadow {

struct RegisterInfo {
  bool ok=false;
  std::string method="NONE";
  int n=0;
  int inliers=0;
  int cells=0;
  double residual=999.0;
};

struct RegisterResult {
  std::vector<cv::Point2f> a;
  std::vector<cv::Point2f> b;
  RegisterInfo info;
};

inline cv::Mat roiMask(const cv::Mat& im) {
  cv::Mat m(im.size(),CV_8UC1,cv::Scalar(0));
  const int x0=(int)(im.cols*0.10), y0=(int)(im.rows*0.44);
  const int x1=(int)(im.cols*0.90), y1=(int)(im.rows*0.96);
  if(x1>x0 && y1>y0) m(cv::Rect(x0,y0,x1-x0,y1-y0)).setTo(255);
  return m;
}

inline std::vector<cv::Point2f> features(const cv::Mat& im) {
  const cv::Mat mask=roiMask(im);
  std::vector<cv::Point2f> out;
  out.reserve(432);
  // Literal Astra grid for 640x480: 4 columns x 3 rows, 160x160 cells.
  for(int gy=0;gy<3;gy++) for(int gx=0;gx<4;gx++) {
    const int x0=gx*160, y0=gy*160;
    if(x0>=im.cols || y0>=im.rows) continue;
    const int w=std::min(160,im.cols-x0), h=std::min(160,im.rows-y0);
    cv::Mat cellMask(im.size(),CV_8UC1,cv::Scalar(0));
    cv::Mat src=mask(cv::Rect(x0,y0,w,h));
    src.copyTo(cellMask(cv::Rect(x0,y0,w,h)));
    std::vector<cv::Point2f> p;
    cv::goodFeaturesToTrack(im,p,36,0.01,7.0,cellMask);
    out.insert(out.end(),p.begin(),p.end());
  }
  return out;
}

inline std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> lk(
    const cv::Mat& a,const cv::Mat& b) {
  std::vector<cv::Point2f> p=features(a),q,back;
  if(p.size()<8) return {};
  std::vector<uchar> s,sb; std::vector<float> e,eb;
  const cv::Size win(31,31);
  const cv::TermCriteria crit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.01);
  cv::calcOpticalFlowPyrLK(a,b,p,q,s,e,win,4,crit);
  if(q.empty()) return {};
  cv::calcOpticalFlowPyrLK(b,a,q,back,sb,eb,win,4,crit);
  if(back.empty()) return {};
  std::vector<cv::Point2f> pa,pb;
  for(size_t i=0;i<p.size() && i<q.size() && i<back.size();++i) {
    if(!s[i] || !sb[i]) continue;
    if(cv::norm(back[i]-p[i])>=0.8) continue;
    if(!(q[i].x>32 && q[i].x<608 && q[i].y>205 && q[i].y<472)) continue;
    pa.push_back(p[i]); pb.push_back(q[i]);
  }
  return {pa,pb};
}

inline RegisterResult fit(const std::vector<cv::Point2f>& a,
                          const std::vector<cv::Point2f>& b,
                          const std::string& method) {
  RegisterResult r; r.info.method=method; r.info.n=(int)a.size();
  if(a.size()<8 || b.size()!=a.size()) return r;
  cv::Mat mask;
  cv::Mat H=cv::findHomography(a,b,cv::RANSAC,1.0,mask,2000,0.999);
  if(H.empty() || mask.empty()) return r;
  std::vector<cv::Point2f> pred;
  cv::perspectiveTransform(a,pred,H);
  std::vector<double> err;
  std::set<std::pair<int,int>> cells;
  for(size_t i=0;i<a.size();++i) {
    if(!mask.at<uchar>((int)i)) continue;
    r.a.push_back(a[i]); r.b.push_back(b[i]);
    err.push_back(cv::norm(pred[i]-b[i]));
    cells.emplace((int)(a[i].x/160.0),(int)(a[i].y/80.0));
  }
  r.info.inliers=(int)r.a.size(); r.info.cells=(int)cells.size();
  if(!err.empty()) {
    const size_t k=err.size()/2;
    std::nth_element(err.begin(),err.begin()+k,err.end());
    r.info.residual=err[k];
    if((err.size()%2)==0) {
      const double hi=r.info.residual;
      std::nth_element(err.begin(),err.begin()+k-1,err.begin()+k);
      r.info.residual=0.5*(hi+err[k-1]);
    }
  }
  r.info.ok=r.info.inliers>=24 && r.info.cells>=4 && r.info.residual<0.65;
  return r;
}

inline std::pair<std::vector<cv::Point2f>,std::vector<cv::Point2f>> sift(
    const cv::Mat& a,const cv::Mat& b) {
  auto detector=cv::SIFT::create(1200,3,0.015,12);
  std::vector<cv::KeyPoint> ka,kb; cv::Mat da,db;
  detector->detectAndCompute(a,roiMask(a),ka,da);
  detector->detectAndCompute(b,roiMask(b),kb,db);
  if(da.empty() || db.empty()) return {};
  cv::BFMatcher matcher;
  std::vector<std::vector<cv::DMatch>> fwd,rev;
  matcher.knnMatch(da,db,fwd,2); matcher.knnMatch(db,da,rev,2);
  std::set<std::pair<int,int>> revgood;
  for(const auto& z:rev) if(z.size()==2 && z[0].distance<0.72f*z[1].distance)
    revgood.emplace(z[0].trainIdx,z[0].queryIdx);
  std::vector<cv::Point2f> pa,pb;
  for(const auto& z:fwd) if(z.size()==2 && z[0].distance<0.72f*z[1].distance &&
      revgood.count({z[0].queryIdx,z[0].trainIdx})) {
    pa.push_back(ka[z[0].queryIdx].pt); pb.push_back(kb[z[0].trainIdx].pt);
  }
  return {pa,pb};
}

inline RegisterResult registerFrames(const cv::Mat& a,const cv::Mat& b,bool force_sift=false) {
  auto lp=lk(a,b);
  RegisterResult best=fit(lp.first,lp.second,"LK4_FB");
  if(!best.info.ok || force_sift) {
    auto sp=sift(a,b);
    RegisterResult sr=fit(sp.first,sp.second,"SIFT_RECOVERY");
    if(sr.info.ok && (!best.info.ok || sr.info.inliers>best.info.inliers)) best=std::move(sr);
  }
  return best;
}

} // namespace astra_shadow
