// Offline parity gate for Astra metric().
// Input is a deterministic CSV of already accepted correspondences exported by
// astra_metric_export.py. This deliberately isolates the Python->C++ metric port
// from frontend differences.
#include <opencv2/opencv.hpp>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using V3=cv::Vec3d; using M3=cv::Matx33d;
struct Row{int interval; double roll0,pitch0,yaw0,roll1,pitch1,yaw1,range0; float ax,ay,bx,by;};
static std::vector<std::string> split(const std::string&s){std::vector<std::string>v;std::stringstream q(s);std::string x;while(std::getline(q,x,','))v.push_back(x);return v;}
static M3 rot(double r,double p,double y){double cr=cos(r),sr=sin(r),cp=cos(p),sp=sin(p),cy=cos(y),sy=sin(y);return {cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr};}
static double median(std::vector<double> v){if(v.empty())return 0;size_t n=v.size()/2;std::nth_element(v.begin(),v.begin()+n,v.end());double a=v[n];if(v.size()%2)return a;std::nth_element(v.begin(),v.begin()+n-1,v.end());return .5*(a+v[n-1]);}
static V3 metric(const std::vector<Row>& q){
  static const cv::Mat K=(cv::Mat_<double>(3,3)<<568.5317075216523,0,315.98271077441063,0,569.6800556286586,239.8814858910064,0,0,1);
  static const cv::Mat D=(cv::Mat_<double>(1,5)<<.07356919219402849,-.095253893789117,-.0108105307571873,-.002284337357697,.08217740080275748);
  const M3 BC(0,-1,0,1,0,0,0,0,1); const V3 CAM(.0625,0,.05),LUNA(.0855,0,.055),normal(0,0,1);
  M3 Rb0=rot(q[0].roll0,q[0].pitch0,q[0].yaw0), Rb1=rot(q[0].roll1,q[0].pitch1,q[0].yaw1);
  M3 C0=Rb0*BC,C1=Rb1*BC,R=C1.t()*C0; V3 n=C0.t()*normal;
  double d=normal.dot(Rb0*(LUNA-CAM+V3(0,0,q[0].range0))); if(d<=0)return V3(NAN,NAN,NAN);
  std::vector<cv::Point2f> ap,bp;for(auto&r:q){ap.emplace_back(r.ax,r.ay);bp.emplace_back(r.bx,r.by);} std::vector<cv::Point2f> au,bu;
  cv::undistortPoints(ap,au,K,D);cv::undistortPoints(bp,bu,K,D);
  const int N=q.size();cv::Mat A(2*N,3,CV_64F),z(2*N,1,CV_64F);std::vector<double>w(N,1.0);
  for(int i=0;i<N;i++){V3 a(au[i].x,au[i].y,1),b(bu[i].x,bu[i].y,1);double den=a.dot(n);if(den<.2)return V3(NAN,NAN,NAN);V3 P=R*(a*(d/den));A.at<double>(2*i,0)=1;A.at<double>(2*i,1)=0;A.at<double>(2*i,2)=-b[0];A.at<double>(2*i+1,0)=0;A.at<double>(2*i+1,1)=1;A.at<double>(2*i+1,2)=-b[1];z.at<double>(2*i)=P[0]-b[0]*P[2];z.at<double>(2*i+1)=P[1]-b[1]*P[2];}
  cv::Mat t;
  for(int it=0;it<4;it++){cv::Mat Aw=A.clone(),zw=z.clone();for(int i=0;i<N;i++){double s=sqrt(w[i]);Aw.row(2*i)*=s;Aw.row(2*i+1)*=s;zw.at<double>(2*i)*=s;zw.at<double>(2*i+1)*=s;}cv::solve(Aw,zw,t,cv::DECOMP_SVD);std::vector<double>e(N);for(int i=0;i<N;i++){double ex=z.at<double>(2*i)-(t.at<double>(0)-bu[i].x*t.at<double>(2));double ey=z.at<double>(2*i+1)-(t.at<double>(1)-bu[i].y*t.at<double>(2));e[i]=hypot(ex,ey);}double me=median(e);std::vector<double>ad;for(double x:e)ad.push_back(fabs(x-me));double sc=std::max(.00003,1.4826*median(ad));for(int i=0;i<N;i++)w[i]=std::min(1.0,2.5*sc/std::max(e[i],1e-9));}
  V3 tv(t.at<double>(0),t.at<double>(1),t.at<double>(2));return C1*tv-(Rb1-Rb0)*CAM;
}
int main(int ac,char**av){if(ac!=2){std::cerr<<"usage: astra_metric_cpp_parity metric_inputs.csv\n";return 2;}std::ifstream f(av[1]);if(!f){std::cerr<<"cannot open input\n";return 2;}std::string s;std::getline(f,s);std::vector<Row> all;while(std::getline(f,s)){auto v=split(s);if(v.size()!=12)continue;Row r;r.interval=stoi(v[0]);r.roll0=stod(v[1]);r.pitch0=stod(v[2]);r.yaw0=stod(v[3]);r.roll1=stod(v[4]);r.pitch1=stod(v[5]);r.yaw1=stod(v[6]);r.range0=stod(v[7]);r.ax=stof(v[8]);r.ay=stof(v[9]);r.bx=stof(v[10]);r.by=stof(v[11]);all.push_back(r);}V3 total(0,0,0);int n=0;for(size_t i=0;i<all.size();){size_t j=i+1;while(j<all.size()&&all[j].interval==all[i].interval)j++;std::vector<Row>q(all.begin()+i,all.begin()+j);V3 d=metric(q);if(!std::isfinite(d[0])){std::cerr<<"bad interval "<<all[i].interval<<"\n";return 3;}total+=d;n++;i=j;}double mag=hypot(total[0],total[1])*1000.;std::cout<<std::fixed<<std::setprecision(9)<<"ASTRA C++ METRIC PARITY\nintervals="<<n<<"\nN/E = ("<<total[0]*1000<<", "<<total[1]*1000<<") mm\nmagnitude = "<<mag<<" mm\n";return 0;}
