#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

namespace astra_metric {

using V3=cv::Vec3d;
using M3=cv::Matx33d;

inline M3 rotation(double r,double p,double y){
  const double cr=std::cos(r),sr=std::sin(r),cp=std::cos(p),sp=std::sin(p),cy=std::cos(y),sy=std::sin(y);
  return M3(cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,
            sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,
            -sp,cp*sr,cp*cr);
}

inline double median(std::vector<double> v){
  if(v.empty()) return 0.0;
  const size_t n=v.size()/2;
  std::nth_element(v.begin(),v.begin()+n,v.end());
  const double hi=v[n];
  if(v.size()%2) return hi;
  std::nth_element(v.begin(),v.begin()+n-1,v.begin()+n);
  return 0.5*(hi+v[n-1]);
}

// Literal reusable C++ port of Astra estimator/estimator.py::metric().
// pa/pb are already accepted Astra frontend inlier correspondences.
// Rb0/Rb1 are body-FRD -> NED rotations at the two camera timestamps.
// range0 is TF-Luna range at the anchor timestamp.
inline V3 metric(const std::vector<cv::Point2f>& pa,
                 const std::vector<cv::Point2f>& pb,
                 const M3& Rb0,const M3& Rb1,double range0){
  if(pa.size()<8 || pb.size()!=pa.size() || !(range0>0.0))
    return V3(NAN,NAN,NAN);

  static const cv::Mat K=(cv::Mat_<double>(3,3)<<
    568.5317075216523,0,315.98271077441063,
    0,569.6800556286586,239.8814858910064,
    0,0,1);
  static const cv::Mat D=(cv::Mat_<double>(1,5)<<
    .07356919219402849,-.095253893789117,-.0108105307571873,
    -.002284337357697,.08217740080275748);
  static const M3 BC(0,-1,0, 1,0,0, 0,0,1);
  static const V3 CAM(.0625,0,.05),LUNA(.0855,0,.055),normal(0,0,1);

  const M3 C0=Rb0*BC,C1=Rb1*BC,R=C1.t()*C0;
  const V3 n=C0.t()*normal;
  const double d=normal.dot(Rb0*(LUNA-CAM+V3(0,0,range0)));
  if(d<=0.0) return V3(NAN,NAN,NAN);

  std::vector<cv::Point2f> au,bu;
  cv::undistortPoints(pa,au,K,D);
  cv::undistortPoints(pb,bu,K,D);

  const int N=(int)pa.size();
  cv::Mat A(2*N,3,CV_64F),z(2*N,1,CV_64F);
  std::vector<double> w(N,1.0);
  for(int i=0;i<N;i++){
    const V3 a(au[i].x,au[i].y,1.0),b(bu[i].x,bu[i].y,1.0);
    const double den=a.dot(n);
    if(den<.2) return V3(NAN,NAN,NAN);
    const V3 P=R*(a*(d/den));
    A.at<double>(2*i,0)=1; A.at<double>(2*i,1)=0; A.at<double>(2*i,2)=-b[0];
    A.at<double>(2*i+1,0)=0; A.at<double>(2*i+1,1)=1; A.at<double>(2*i+1,2)=-b[1];
    z.at<double>(2*i)=P[0]-b[0]*P[2];
    z.at<double>(2*i+1)=P[1]-b[1]*P[2];
  }

  cv::Mat t;
  for(int it=0;it<4;it++){
    cv::Mat Aw=A.clone(),zw=z.clone();
    for(int i=0;i<N;i++){
      const double s=std::sqrt(w[i]);
      Aw.row(2*i)*=s; Aw.row(2*i+1)*=s;
      zw.at<double>(2*i)*=s; zw.at<double>(2*i+1)*=s;
    }
    if(!cv::solve(Aw,zw,t,cv::DECOMP_SVD)) return V3(NAN,NAN,NAN);
    std::vector<double> e(N);
    for(int i=0;i<N;i++){
      const double ex=z.at<double>(2*i)-(t.at<double>(0)-bu[i].x*t.at<double>(2));
      const double ey=z.at<double>(2*i+1)-(t.at<double>(1)-bu[i].y*t.at<double>(2));
      e[i]=std::hypot(ex,ey);
    }
    const double me=median(e);
    std::vector<double> ad; ad.reserve(e.size());
    for(double x:e) ad.push_back(std::abs(x-me));
    const double sc=std::max(.00003,1.4826*median(ad));
    for(int i=0;i<N;i++) w[i]=std::min(1.0,2.5*sc/std::max(e[i],1e-9));
  }

  const V3 tv(t.at<double>(0),t.at<double>(1),t.at<double>(2));
  return C1*tv-(Rb1-Rb0)*CAM;
}

} // namespace astra_metric
