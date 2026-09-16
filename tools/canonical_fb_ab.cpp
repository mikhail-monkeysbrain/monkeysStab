// Fast A/B: current estimator vs forward-backward LK consistency on canonical dataset.
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
struct Meta{long long frame=0,cam=0,mono=0;size_t size=0;};
struct Step{bool valid=false;int features=0,tracked=0,inliers=0;double du=0,dv=0;};
struct Sum{double x=0,y=0;long long valid=0,invalid=0;};
static std::vector<std::string> split(const std::string&s){std::vector<std::string>v;std::stringstream q(s);std::string x;while(std::getline(q,x,','))v.push_back(x);return v;}
static Step est(const cv::Mat&p,const cv::Mat&c,double dt,const cv::Mat&K,const cv::Mat&D,bool fb){
 Step o;if(!(dt>0&&dt<.2))return o;int x0=.20*p.cols,y0=.32*p.rows,x1=.80*p.cols,y1=.90*p.rows;std::vector<cv::Point2f>a,b;constexpr int G=3;int per=(500+8)/9;
 for(int gy=0;gy<G;gy++)for(int gx=0;gx<G;gx++){cv::Rect z(x0+(x1-x0)*gx/G,y0+(y1-y0)*gy/G,(x1-x0)*(gx+1)/G-(x1-x0)*gx/G,(y1-y0)*(gy+1)/G-(y1-y0)*gy/G);std::vector<cv::Point2f>q;cv::goodFeaturesToTrack(p(z),q,per,.01,7);for(auto t:q){t.x+=z.x;t.y+=z.y;a.push_back(t);if(a.size()>=500)break;}if(a.size()>=500)break;}
 o.features=a.size();if(a.size()<30)return o;std::vector<uchar>st;std::vector<float>er;cv::calcOpticalFlowPyrLK(p,c,a,b,st,er,{21,21},3,cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,.01),0,1e-4);
 std::vector<cv::Point2f>A,B;for(size_t i=0;i<a.size();i++)if(st[i]){A.push_back(a[i]);B.push_back(b[i]);}
 if(fb&&!A.empty()){std::vector<cv::Point2f>back;std::vector<uchar>s2;std::vector<float>e2;cv::calcOpticalFlowPyrLK(c,p,B,back,s2,e2,{21,21},3,cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,.01),0,1e-4);std::vector<cv::Point2f>aa,bb;for(size_t i=0;i<A.size();i++)if(s2[i]&&cv::norm(back[i]-A[i])<=2.0){aa.push_back(A[i]);bb.push_back(B[i]);}A.swap(aa);B.swap(bb);}
 o.tracked=A.size();if(A.size()<20)return o;cv::Mat mask;cv::findHomography(A,B,cv::RANSAC,2.,mask,350,.99);if(mask.empty())return o;std::vector<cv::Point2f>ai,bi;for(size_t i=0;i<A.size();i++)if(mask.at<uchar>(i)){ai.push_back(A[i]);bi.push_back(B[i]);}o.inliers=ai.size();if(ai.size()<20)return o;
 std::vector<cv::Point2f>au,bu;cv::undistortPoints(ai,au,K,D);cv::undistortPoints(bi,bu,K,D);cv::Mat M(ai.size()*2,4,CV_64F),v(ai.size()*2,1,CV_64F);for(size_t k=0;k<ai.size();k++){double x=au[k].x,y=au[k].y,dx=bu[k].x-x,dy=bu[k].y-y;int r=2*k;M.at<double>(r,0)=1;M.at<double>(r,1)=0;M.at<double>(r,2)=x;M.at<double>(r,3)=-y;v.at<double>(r)=dx;M.at<double>(r+1,0)=0;M.at<double>(r+1,1)=1;M.at<double>(r+1,2)=y;M.at<double>(r+1,3)=x;v.at<double>(r+1)=dy;}cv::Mat s;if(!cv::solve(M,v,s,cv::DECOMP_SVD))return o;o.du=s.at<double>(0);o.dv=s.at<double>(1);o.valid=std::hypot(o.dv/dt,-o.du/dt)<4.;return o;
}
static Sum run(const std::vector<cv::Mat>&im,const std::vector<Meta>&m,int lo,int hi,bool fb,const char*name){
 Sum s;int n=hi-lo,last=-1;auto t=std::chrono::steady_clock::now();for(int i=lo+1;i<=hi;i++){Step q=est(im[i-1],im[i],(m[i].cam-m[i-1].cam)*1e-9,cv::Mat(),cv::Mat(),fb);if(q.valid){s.x+=q.du;s.y+=q.dv;s.valid++;}else s.invalid++;int pc=100*(i-lo)/n;if(pc/10!=last/10){last=pc;std::cerr<<"\r"<<name<<" "<<pc<<"%"<<std::flush;}}std::cerr<<"\r"<<name<<" 100%\n";return s;
}
static void pr(const char*n,const Sum&s){std::cout<<std::fixed<<std::setprecision(9)<<n<<" X="<<s.x<<" Y="<<s.y<<" mag="<<std::hypot(s.x,s.y)<<" valid/invalid="<<s.valid<<"/"<<s.invalid<<"\n";}
int main(int ac,char**av){if(ac!=2){std::cerr<<"usage: canonical_fb_ab DATASET\n";return 2;}cv::setNumThreads(1);std::string d=av[1];std::ifstream fc(d+"/frames.csv"),bin(d+"/frames.mjpgbin",std::ios::binary),fl(d+"/optical_flow_mavlink.csv");std::string l;std::vector<Meta>m;std::getline(fc,l);while(std::getline(fc,l)){auto v=split(l);m.push_back({std::stoll(v[0]),std::stoll(v[1]),std::stoll(v[2]),(size_t)std::stoull(v[3])});}std::vector<cv::Mat>im;for(auto&x:m){uint64_t ts;uint32_t z;bin.read((char*)&ts,8);bin.read((char*)&z,4);std::vector<uchar>j(z);bin.read((char*)j.data(),z);im.push_back(cv::imdecode(j,0));}
 std::getline(fl,l);auto h=split(l);int F=-1,E=-1;for(int i=0;i<h.size();i++){if(h[i]=="frame")F=i;if(h[i]=="return_event")E=i;}long long af=-1,bf=-1,cf=-1;while(std::getline(fl,l)){auto v=split(l);int e=std::stoi(v[E]);long long f=std::stoll(v[F]);if(e==1&&af<0)af=f;if(e==2&&bf<0)bf=f;if(e==3&&cf<0)cf=f;}auto ix=[&](long long f){for(int i=0;i<m.size();i++)if(m[i].frame==f)return i;return -1;};int a=ix(af),b=ix(bf),c=ix(cf);
 double fs=.931;cv::Mat K=(cv::Mat_<double>(3,3)<<568.53170752165227*fs,0,315.98271077441063,0,569.68005562865858*fs,239.88148589100641,0,0,1),D=(cv::Mat_<double>(1,5)<<.073569192194028493,-.095253893789117,-.010810530757187299,-.0022843373576970235,.082177400802757483);
 // est currently receives empty K/D above by mistake prevention: replace calls locally via lambda implementation is required.
 auto R=[&](int lo,int hi,bool fb,const char*n){Sum s;int N=hi-lo,last=-1;for(int i=lo+1;i<=hi;i++){Step q=est(im[i-1],im[i],(m[i].cam-m[i-1].cam)*1e-9,K,D,fb);if(q.valid){s.x+=q.du;s.y+=q.dv;s.valid++;}else s.invalid++;int pc=100*(i-lo)/N;if(pc/10!=last/10){last=pc;std::cerr<<"\r"<<n<<" "<<pc<<"%"<<std::flush;}}std::cerr<<"\r"<<n<<" 100%\n";return s;};
 std::cout<<"===== CURRENT vs CURRENT+FB =====\n";Sum acur=R(a,b,false,"A->B current"),afb=R(a,b,true,"A->B FB"),bcur=R(b,c,false,"B->A current"),bfb=R(b,c,true,"B->A FB");pr("A->B CURRENT",acur);pr("A->B FB     ",afb);pr("B->A CURRENT",bcur);pr("B->A FB     ",bfb);
 auto closure=[&](const char*n,const Sum&A,const Sum&B){double ma=std::hypot(A.x,A.y),cx=A.x+B.x,cy=A.y+B.y;std::cout<<n<<" closure="<<std::hypot(cx,cy)<<" = "<<100*std::hypot(cx,cy)/ma<<"% of |AB|; BA/AB="<<std::hypot(B.x,B.y)/ma<<"\n";};closure("CURRENT",acur,bcur);closure("FB     ",afb,bfb);return 0;}
