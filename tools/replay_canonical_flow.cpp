// Forensic replay for canonical capture. Reference estimator: cc58fef13dc14f8e65ad16fd7f69468006925a8a
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct Meta { long long frame=0, cam=0, mono=0; size_t size=0; };
struct Step { bool valid=false; int reason=0,features=0,tracked=0,inliers=0; double du_px=0,dv_px=0,du=0,dv=0,scale=0,yaw=0; double sim_rms=0,aff_rms=0,aff_tx=0,aff_ty=0,aff_a11=0,aff_a12=0,aff_a21=0,aff_a22=0; };
static double med(std::vector<double> v){ if(v.empty()) return 0; size_t n=v.size()/2; std::nth_element(v.begin(),v.begin()+n,v.end()); double x=v[n]; if(v.size()%2==0){std::nth_element(v.begin(),v.begin()+n-1,v.end());x=(x+v[n-1])*0.5;} return x; }
static std::vector<std::string> split(const std::string&s){std::vector<std::string>v;std::stringstream q(s);std::string x;while(std::getline(q,x,','))v.push_back(x);return v;}

static Step estimate(const cv::Mat& prev,const cv::Mat& curr,double dt,const cv::Mat& K,const cv::Mat&D){
 Step o;if(prev.empty()||curr.empty()||!(dt>0&&dt<0.2)){o.reason=1;return o;}
 const double rx0=.20,ry0=.32,rx1=.80,ry1=.90; const int maxf=500;
 int x0=std::clamp((int)std::lround(rx0*prev.cols),0,prev.cols-1),y0=std::clamp((int)std::lround(ry0*prev.rows),0,prev.rows-1);
 int x1=std::clamp((int)std::lround(rx1*prev.cols),x0+1,prev.cols),y1=std::clamp((int)std::lround(ry1*prev.rows),y0+1,prev.rows);
 std::vector<cv::Point2f>p0,p1; constexpr int G=3; int per=std::max(1,(maxf+G*G-1)/(G*G)); p0.reserve(maxf);
 for(int gy=0;gy<G;gy++){int cy0=y0+(y1-y0)*gy/G,cy1=y0+(y1-y0)*(gy+1)/G;for(int gx=0;gx<G;gx++){int cx0=x0+(x1-x0)*gx/G,cx1=x0+(x1-x0)*(gx+1)/G;if(cx1<=cx0||cy1<=cy0)continue;cv::Rect cell(cx0,cy0,cx1-cx0,cy1-cy0);std::vector<cv::Point2f>local;cv::goodFeaturesToTrack(prev(cell),local,per,.01,7);for(auto p:local){p.x+=cell.x;p.y+=cell.y;p0.push_back(p);if((int)p0.size()>=maxf)break;}if((int)p0.size()>=maxf)break;}if((int)p0.size()>=maxf)break;}
 if(p0.size()<30){cv::Mat m(prev.size(),CV_8UC1,cv::Scalar(0));int fx0=std::clamp((int)std::lround(.05*prev.cols),0,prev.cols-1),fy0=std::clamp((int)std::lround(.25*prev.rows),0,prev.rows-1),fx1=std::clamp((int)std::lround(.95*prev.cols),fx0+1,prev.cols),fy1=std::clamp((int)std::lround(.98*prev.rows),fy0+1,prev.rows);m(cv::Rect(fx0,fy0,fx1-fx0,fy1-fy0)).setTo(255);std::vector<cv::Point2f>pf;cv::goodFeaturesToTrack(prev,pf,maxf,.005,5,m);if(pf.size()>p0.size())p0.swap(pf);}
 o.features=p0.size();if(p0.size()<30){o.reason=2;return o;}
 std::vector<uchar>st;std::vector<float>err;cv::calcOpticalFlowPyrLK(prev,curr,p0,p1,st,err,{21,21},3,cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,.01),0,1e-4);
 std::vector<cv::Point2f>a,b;for(size_t i=0;i<p0.size();i++)if(st[i]){a.push_back(p0[i]);b.push_back(p1[i]);}o.tracked=a.size();if(a.size()<20){o.reason=3;return o;}
 cv::Mat mask;cv::findHomography(a,b,cv::RANSAC,2.0,mask,350,.99);if(mask.empty()){o.reason=4;return o;}std::vector<cv::Point2f>ai,bi;for(size_t i=0;i<a.size();i++)if(mask.at<uchar>((int)i)){ai.push_back(a[i]);bi.push_back(b[i]);}o.inliers=ai.size();if(ai.size()<20){o.reason=5;return o;}
 std::vector<cv::Point2f>au,bu;cv::undistortPoints(ai,au,K,D);cv::undistortPoints(bi,bu,K,D);std::vector<double>dup,dvp;cv::Mat A(ai.size()*2,4,CV_64F),bb(ai.size()*2,1,CV_64F);
 for(size_t k=0;k<ai.size();k++){double x=au[k].x,y=au[k].y,du=bu[k].x-au[k].x,dv=bu[k].y-au[k].y;dup.push_back(bi[k].x-ai[k].x);dvp.push_back(bi[k].y-ai[k].y);int r=2*k;A.at<double>(r,0)=1;A.at<double>(r,1)=0;A.at<double>(r,2)=x;A.at<double>(r,3)=-y;bb.at<double>(r)=du;A.at<double>(r+1,0)=0;A.at<double>(r+1,1)=1;A.at<double>(r+1,2)=y;A.at<double>(r+1,3)=x;bb.at<double>(r+1)=dv;}
 cv::Mat sol;if(!cv::solve(A,bb,sol,cv::DECOMP_SVD)){o.reason=5;return o;}o.du=sol.at<double>(0);o.dv=sol.at<double>(1);o.scale=sol.at<double>(2)/dt;o.yaw=sol.at<double>(3)/dt;
 // Shadow forensic only: fit a full 6-parameter affine displacement field to the
 // EXACT SAME production RANSAC inliers. It never changes the production result.
 // du = tx + a11*x + a12*y ; dv = ty + a21*x + a22*y
 cv::Mat Af((int)ai.size()*2,6,CV_64F),bf=bb.clone();
 for(size_t k=0;k<ai.size();++k){double x=au[k].x,y=au[k].y;int r=(int)(2*k);
   Af.at<double>(r,0)=1;Af.at<double>(r,1)=0;Af.at<double>(r,2)=x;Af.at<double>(r,3)=y;Af.at<double>(r,4)=0;Af.at<double>(r,5)=0;
   Af.at<double>(r+1,0)=0;Af.at<double>(r+1,1)=1;Af.at<double>(r+1,2)=0;Af.at<double>(r+1,3)=0;Af.at<double>(r+1,4)=x;Af.at<double>(r+1,5)=y;
 }
 cv::Mat sf; if(cv::solve(Af,bf,sf,cv::DECOMP_SVD)&&sf.rows==6){
   o.aff_tx=sf.at<double>(0);o.aff_ty=sf.at<double>(1);o.aff_a11=sf.at<double>(2);o.aff_a12=sf.at<double>(3);o.aff_a21=sf.at<double>(4);o.aff_a22=sf.at<double>(5);
   double ss=0,sa=0; for(size_t k=0;k<ai.size();++k){double x=au[k].x,y=au[k].y,du=bu[k].x-x,dv=bu[k].y-y;
     double sdu=o.du+(o.scale*dt)*x-(o.yaw*dt)*y, sdv=o.dv+(o.scale*dt)*y+(o.yaw*dt)*x;
     double adu=o.aff_tx+o.aff_a11*x+o.aff_a12*y, adv=o.aff_ty+o.aff_a21*x+o.aff_a22*y;
     ss+=(du-sdu)*(du-sdu)+(dv-sdv)*(dv-sdv); sa+=(du-adu)*(du-adu)+(dv-adv)*(dv-adv);
   }
   o.sim_rms=std::sqrt(ss/std::max<size_t>(1,2*ai.size()));o.aff_rms=std::sqrt(sa/std::max<size_t>(1,2*ai.size()));
 }
 o.du_px=med(dup);o.dv_px=med(dvp);o.valid=std::hypot(o.dv/dt,-o.du/dt)<4.0;if(!o.valid)o.reason=6;return o;
}
int main(int argc,char**argv){
 if(argc!=2){std::cerr<<"usage: canonical_cpp_replay DATASET_DIR\n";return 2;}cv::setNumThreads(1);std::string d=argv[1];
 std::ifstream fc(d+"/frames.csv"),fb(d+"/frames.mjpgbin",std::ios::binary),flow(d+"/optical_flow_mavlink.csv");if(!fc||!fb||!flow){std::cerr<<"missing dataset files\n";return 2;}
 std::vector<Meta>ms;std::string line;std::getline(fc,line);while(std::getline(fc,line)){auto v=split(line);if(v.size()>=4)ms.push_back({std::stoll(v[0]),std::stoll(v[1]),std::stoll(v[2]),(size_t)std::stoull(v[3])});}
 std::getline(flow,line);auto hdr=split(line);int ifr=-1,iva=-1,ife=-1,itr=-1,iin=-1,idu=-1,idv=-1;for(int i=0;i<(int)hdr.size();i++){if(hdr[i]=="frame")ifr=i;else if(hdr[i]=="valid")iva=i;else if(hdr[i]=="features")ife=i;else if(hdr[i]=="tracked")itr=i;else if(hdr[i]=="inliers")iin=i;else if(hdr[i]=="du_norm")idu=i;else if(hdr[i]=="dv_norm")idv=i;}
 struct Prod{int valid=0,fe=0,tr=0,in=0;double du=0,dv=0;};std::vector<Prod>prod(ms.size()+1);while(std::getline(flow,line)){auto v=split(line);if(ifr<0||ifr>=(int)v.size())continue;long long f=std::stoll(v[ifr]);if(f>=0&&f<(long long)prod.size())prod[f]={std::stoi(v[iva]),std::stoi(v[ife]),std::stoi(v[itr]),std::stoi(v[iin]),std::stod(v[idu]),std::stod(v[idv])};}
 double fs=.931;cv::Mat K=(cv::Mat_<double>(3,3)<<568.53170752165227*fs,0,315.98271077441063,0,569.68005562865858*fs,239.88148589100641,0,0,1);cv::Mat D=(cv::Mat_<double>(1,5)<<.073569192194028493,-.095253893789117,-.010810530757187299,-.0022843373576970235,.082177400802757483);
 std::ofstream out(d+"/canonical_cpp_replay.csv");out<<"frame,dt_s,replay_valid,prod_valid,replay_features,prod_features,replay_tracked,prod_tracked,replay_inliers,prod_inliers,replay_du_norm,prod_du_norm,replay_dv_norm,prod_dv_norm,sim_rms_norm,aff_rms_norm,aff_improvement_pct,aff_tx,aff_ty,aff_a11,aff_a12,aff_a21,aff_a22\\n";
 cv::Mat prev;long long pts=0;size_t exactF=0,exactT=0,exactI=0,n=0;double maxdu=0,maxdv=0;
 for(auto&m:ms){uint64_t stored_ts=0;uint32_t stored_size=0;fb.read(reinterpret_cast<char*>(&stored_ts),sizeof(stored_ts));fb.read(reinterpret_cast<char*>(&stored_size),sizeof(stored_size));if(!fb){std::cerr<<"short mjpgbin header at frame "<<m.frame<<"\\n";return 3;}if(stored_ts!=(uint64_t)m.cam || stored_size!=(uint32_t)m.size){std::cerr<<"mjpgbin header mismatch at frame "<<m.frame<<": ts "<<stored_ts<<" vs "<<m.cam<<", size "<<stored_size<<" vs "<<m.size<<"\\n";return 3;}std::vector<uchar>jpg(stored_size);fb.read((char*)jpg.data(),jpg.size());if((size_t)fb.gcount()!=jpg.size()){std::cerr<<"short mjpgbin JPEG at frame "<<m.frame<<"\\n";return 3;}cv::Mat cur=cv::imdecode(jpg,cv::IMREAD_GRAYSCALE);if(cur.empty()){std::cerr<<"decode failed "<<m.frame<<"\n";return 3;}if(prev.empty()){prev=cur;pts=m.cam;continue;}double dt=(m.cam-pts)*1e-9;Step s=estimate(prev,cur,dt,K,D);auto&p=prod[m.frame];n++;exactF+=s.features==p.fe;exactT+=s.tracked==p.tr;exactI+=s.inliers==p.in;maxdu=std::max(maxdu,std::abs(s.du-p.du));maxdv=std::max(maxdv,std::abs(s.dv-p.dv));out<<std::setprecision(12)<<m.frame<<","<<dt<<","<<s.valid<<","<<p.valid<<","<<s.features<<","<<p.fe<<","<<s.tracked<<","<<p.tr<<","<<s.inliers<<","<<p.in<<","<<s.du<<","<<p.du<<","<<s.dv<<","<<p.dv<<","<<s.sim_rms<<","<<s.aff_rms<<","<<((s.sim_rms>0)?100.0*(s.sim_rms-s.aff_rms)/s.sim_rms:0.0)<<","<<s.aff_tx<<","<<s.aff_ty<<","<<s.aff_a11<<","<<s.aff_a12<<","<<s.aff_a21<<","<<s.aff_a22<<"\\n";prev=cur;pts=m.cam;}
 std::cout<<"frames compared: "<<n<<"\nfeatures exact: "<<exactF<<"/"<<n<<"\ntracked exact: "<<exactT<<"/"<<n<<"\ninliers exact: "<<exactI<<"/"<<n<<"\nmax |du_norm diff|: "<<maxdu<<"\nmax |dv_norm diff|: "<<maxdv<<"\nCSV: "<<d<<"/canonical_cpp_replay.csv\n";return 0;
}