// Forward/reverse reciprocity test on the exact same saved canonical frames.
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct Meta { long long frame=0, cam=0, mono=0; size_t size=0; };
struct Step { bool valid=false; int features=0,tracked=0,inliers=0; double du=0,dv=0,rate=0; std::string reason="PRECHECK"; };
static std::vector<std::string> split(const std::string&s){std::vector<std::string>v;std::stringstream q(s);std::string x;while(std::getline(q,x,','))v.push_back(x);return v;}

static Step estimate(const cv::Mat& prev,const cv::Mat& curr,double dt,const cv::Mat& K,const cv::Mat&D,bool fb=false){
 Step o;if(prev.empty()||curr.empty()||!(dt>0&&dt<0.2))return o;
 const double rx0=.20,ry0=.32,rx1=.80,ry1=.90; const int maxf=500;
 int x0=std::clamp((int)std::lround(rx0*prev.cols),0,prev.cols-1),y0=std::clamp((int)std::lround(ry0*prev.rows),0,prev.rows-1);
 int x1=std::clamp((int)std::lround(rx1*prev.cols),x0+1,prev.cols),y1=std::clamp((int)std::lround(ry1*prev.rows),y0+1,prev.rows);
 std::vector<cv::Point2f>p0,p1; constexpr int G=3; int per=std::max(1,(maxf+G*G-1)/(G*G)); p0.reserve(maxf);
 for(int gy=0;gy<G;gy++){int cy0=y0+(y1-y0)*gy/G,cy1=y0+(y1-y0)*(gy+1)/G;for(int gx=0;gx<G;gx++){int cx0=x0+(x1-x0)*gx/G,cx1=x0+(x1-x0)*(gx+1)/G;if(cx1<=cx0||cy1<=cy0)continue;cv::Rect cell(cx0,cy0,cx1-cx0,cy1-cy0);std::vector<cv::Point2f>local;cv::goodFeaturesToTrack(prev(cell),local,per,.01,7);for(auto p:local){p.x+=cell.x;p.y+=cell.y;p0.push_back(p);if((int)p0.size()>=maxf)break;}if((int)p0.size()>=maxf)break;}if((int)p0.size()>=maxf)break;}
 if(p0.size()<30){cv::Mat m(prev.size(),CV_8UC1,cv::Scalar(0));int fx0=std::clamp((int)std::lround(.05*prev.cols),0,prev.cols-1),fy0=std::clamp((int)std::lround(.25*prev.rows),0,prev.rows-1),fx1=std::clamp((int)std::lround(.95*prev.cols),fx0+1,prev.cols),fy1=std::clamp((int)std::lround(.98*prev.rows),fy0+1,prev.rows);m(cv::Rect(fx0,fy0,fx1-fx0,fy1-fy0)).setTo(255);std::vector<cv::Point2f>pf;cv::goodFeaturesToTrack(prev,pf,maxf,.005,5,m);if(pf.size()>p0.size())p0.swap(pf);}
 o.features=p0.size(); if(p0.size()<30){o.reason="FEATURES_LT30";return o;}
 std::vector<uchar>st;std::vector<float>err;cv::calcOpticalFlowPyrLK(prev,curr,p0,p1,st,err,{21,21},3,cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,.01),0,1e-4);
 std::vector<cv::Point2f>a,b;for(size_t i=0;i<p0.size();i++)if(st[i]){a.push_back(p0[i]);b.push_back(p1[i]);}
 if(fb&&!a.empty()){std::vector<cv::Point2f>back;std::vector<uchar>st2;std::vector<float>er2;cv::calcOpticalFlowPyrLK(curr,prev,b,back,st2,er2,{21,21},3,cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,.01),0,1e-4);std::vector<cv::Point2f>aa,bb;for(size_t i=0;i<a.size();i++)if(st2[i]&&cv::norm(back[i]-a[i])<=2.0){aa.push_back(a[i]);bb.push_back(b[i]);}a.swap(aa);b.swap(bb);}
 o.tracked=a.size();if(a.size()<20){o.reason="TRACKED_LT20";return o;}
 cv::Mat mask;cv::findHomography(a,b,cv::RANSAC,2.0,mask,350,.99);if(mask.empty()){o.reason="HOMOGRAPHY_EMPTY";return o;}std::vector<cv::Point2f>ai,bi;for(size_t i=0;i<a.size();i++)if(mask.at<uchar>((int)i)){ai.push_back(a[i]);bi.push_back(b[i]);}o.inliers=ai.size();if(ai.size()<20){o.reason="INLIERS_LT20";return o;}
 std::vector<cv::Point2f>au,bu;cv::undistortPoints(ai,au,K,D);cv::undistortPoints(bi,bu,K,D);cv::Mat A(ai.size()*2,4,CV_64F),bb(ai.size()*2,1,CV_64F);
 for(size_t k=0;k<ai.size();k++){double x=au[k].x,y=au[k].y,du=bu[k].x-au[k].x,dv=bu[k].y-au[k].y;int r=2*k;A.at<double>(r,0)=1;A.at<double>(r,1)=0;A.at<double>(r,2)=x;A.at<double>(r,3)=-y;bb.at<double>(r)=du;A.at<double>(r+1,0)=0;A.at<double>(r+1,1)=1;A.at<double>(r+1,2)=y;A.at<double>(r+1,3)=x;bb.at<double>(r+1)=dv;}
 cv::Mat sol;if(!cv::solve(A,bb,sol,cv::DECOMP_SVD)){o.reason="SOLVE_FAIL";return o;}o.du=sol.at<double>(0);o.dv=sol.at<double>(1);o.rate=std::hypot(o.dv/dt,-o.du/dt);o.valid=o.rate<4.0;o.reason=o.valid?"OK":"RATE_GE4";return o;
}
struct Sum{double x=0,y=0;long long valid=0,invalid=0;};
static void forensic(const std::vector<cv::Mat>&im,const std::vector<Meta>&m,int lo,int hi,const cv::Mat&K,const cv::Mat&D){
 struct R{int i;double e,fx,fy,rx,ry,fr,rr;int fv,rv,ffi,rfi,ft,rt,fi,ri;std::string freason,rreason;};std::vector<R>v;double total=0;
 for(int i=lo+1;i<=hi;i++){double dt=(m[i].cam-m[i-1].cam)*1e-9;Step f=estimate(im[i-1],im[i],dt,K,D),r=estimate(im[i],im[i-1],dt,K,D);double ex=(f.valid?f.du:0)+(r.valid?r.du:0),ey=(f.valid?f.dv:0)+(r.valid?r.dv:0),e=std::hypot(ex,ey);total+=e;v.push_back({i,e,f.du,f.dv,r.du,r.dv,f.rate,r.rate,f.valid,r.valid,f.features,r.features,f.tracked,r.tracked,f.inliers,r.inliers,f.reason,r.reason});}
 std::sort(v.begin(),v.end(),[](const R&a,const R&b){return a.e>b.e;});
 std::cout<<"\\n===== B->A TOP SAME-PAIR RECIPROCITY ERRORS =====\\n";
 std::cout<<"rank frame0 frame1 err f_valid r_valid f_reason r_reason f_feat r_feat f_trk r_trk f_inl r_inl f_rate r_rate f_du f_dv r_du r_dv\\n";
 double top10=0,top50=0;for(size_t k=0;k<v.size();k++){if(k<10)top10+=v[k].e;if(k<50)top50+=v[k].e;if(k<25){auto&q=v[k];std::cout<<k+1<<" "<<m[q.i-1].frame<<" "<<m[q.i].frame<<" "<<q.e<<" "<<q.fv<<" "<<q.rv<<" "<<q.freason<<" "<<q.rreason<<" "<<q.ffi<<" "<<q.rfi<<" "<<q.ft<<" "<<q.rt<<" "<<q.fi<<" "<<q.ri<<" "<<q.fr<<" "<<q.rr<<" "<<q.fx<<" "<<q.fy<<" "<<q.rx<<" "<<q.ry<<"\\n";}}
 std::cout<<"sum pair-error magnitudes: "<<total<<"\\n";
 std::cout<<"top10 share: "<<(total?100*top10/total:NAN)<<" %; top50 share: "<<(total?100*top50/total:NAN)<<" %\\n";
}
static Sum run(const std::vector<cv::Mat>& im,const std::vector<Meta>& m,int lo,int hi,bool reverse,const cv::Mat&K,const cv::Mat&D,bool fb=false){
 Sum s;
 if(!reverse){for(int i=lo+1;i<=hi;i++){double dt=(m[i].cam-m[i-1].cam)*1e-9;Step q=estimate(im[i-1],im[i],dt,K,D,fb);if(q.valid){s.x+=q.du;s.y+=q.dv;s.valid++;}else s.invalid++;}}
 else {for(int i=hi;i>lo;i--){double dt=(m[i].cam-m[i-1].cam)*1e-9;Step q=estimate(im[i],im[i-1],dt,K,D,fb);if(q.valid){s.x+=q.du;s.y+=q.dv;s.valid++;}else s.invalid++;}}
 return s;
}
static void report(const char*name,const Sum&f,const Sum&r){
 double mf=std::hypot(f.x,f.y),mr=std::hypot(r.x,r.y),cx=f.x+r.x,cy=f.y+r.y,cl=std::hypot(cx,cy);
 double dot=f.x*r.x+f.y*r.y,ang=NAN;if(mf>0&&mr>0){double z=std::clamp(dot/(mf*mr),-1.0,1.0);ang=std::acos(z)*180.0/CV_PI;}
 std::cout<<"\n===== "<<name<<" SAME-FRAME FORWARD/REVERSE =====\n";
 std::cout<<std::fixed<<std::setprecision(9);
 std::cout<<"forward: X="<<f.x<<" Y="<<f.y<<" mag="<<mf<<" valid/invalid="<<f.valid<<"/"<<f.invalid<<"\n";
 std::cout<<"reverse: X="<<r.x<<" Y="<<r.y<<" mag="<<mr<<" valid/invalid="<<r.valid<<"/"<<r.invalid<<"\n";
 std::cout<<"reverse/forward magnitude: "<<(mf?mr/mf:NAN)<<"\n";
 std::cout<<"angle: "<<ang<<" deg; deviation from 180: "<<std::abs(180.0-ang)<<" deg\n";
 std::cout<<"closure: X="<<cx<<" Y="<<cy<<" mag="<<cl<<" = "<<(mf?100.0*cl/mf:NAN)<<" % of forward\n";
}
int main(int argc,char**argv){
 if(argc!=2){std::cerr<<"usage: canonical_forward_reverse DATASET_DIR\n";return 2;}cv::setNumThreads(1);std::string d=argv[1];
 std::ifstream fc(d+"/frames.csv"),fb(d+"/frames.mjpgbin",std::ios::binary),flow(d+"/optical_flow_mavlink.csv");if(!fc||!fb||!flow){std::cerr<<"missing dataset files\n";return 2;}
 std::vector<Meta> m;std::string line;std::getline(fc,line);while(std::getline(fc,line)){auto v=split(line);if(v.size()>=4)m.push_back({std::stoll(v[0]),std::stoll(v[1]),std::stoll(v[2]),(size_t)std::stoull(v[3])});}
 std::vector<cv::Mat> im;im.reserve(m.size());for(auto&x:m){uint64_t ts;uint32_t sz;fb.read((char*)&ts,sizeof(ts));fb.read((char*)&sz,sizeof(sz));if(!fb||ts!=(uint64_t)x.cam||sz!=(uint32_t)x.size){std::cerr<<"mjpgbin mismatch frame "<<x.frame<<"\n";return 3;}std::vector<uchar>jpg(sz);fb.read((char*)jpg.data(),sz);cv::Mat z=cv::imdecode(jpg,cv::IMREAD_GRAYSCALE);if(z.empty()){std::cerr<<"decode failed "<<x.frame<<"\n";return 3;}im.push_back(z);}
 std::getline(flow,line);auto h=split(line);int ifr=-1,iev=-1;for(int i=0;i<(int)h.size();i++){if(h[i]=="frame")ifr=i;if(h[i]=="return_event")iev=i;}long long fa=-1,fbf=-1,fcf=-1;while(std::getline(flow,line)){auto v=split(line);if(ifr<0||iev<0||ifr>=(int)v.size()||iev>=(int)v.size())continue;int e=std::stoi(v[iev]);long long fr=std::stoll(v[ifr]);if(e==1&&fa<0)fa=fr;if(e==2&&fbf<0)fbf=fr;if(e==3&&fcf<0)fcf=fr;}
 auto idx=[&](long long fr){for(int i=0;i<(int)m.size();i++)if(m[i].frame==fr)return i;return -1;};int a=idx(fa),b=idx(fbf),c=idx(fcf);if(a<0||b<0||c<0){std::cerr<<"return event frames not found: "<<fa<<" "<<fbf<<" "<<fcf<<"\n";return 4;}
 double fs=.931;cv::Mat K=(cv::Mat_<double>(3,3)<<568.53170752165227*fs,0,315.98271077441063,0,569.68005562865858*fs,239.88148589100641,0,0,1);cv::Mat D=(cv::Mat_<double>(1,5)<<.073569192194028493,-.095253893789117,-.010810530757187299,-.0022843373576970235,.082177400802757483);
 std::cout<<"events frames: A1="<<fa<<" B="<<fbf<<" A2="<<fcf<<"\n";
 Sum abf=run(im,m,a,b,false,K,D),abr=run(im,m,a,b,true,K,D);report("PHYSICAL A->B",abf,abr);
 Sum baf=run(im,m,b,c,false,K,D),bar=run(im,m,b,c,true,K,D);report("PHYSICAL B->A",baf,bar); forensic(im,m,b,c,K,D);
 std::cout<<"\\n===== EXACT ESTIMATOR: BASELINE vs +FB (forward physical legs) =====\\n";
 Sum abfb=run(im,m,a,b,false,K,D,true),bafb=run(im,m,b,c,false,K,D,true);
 auto leg=[&](const char*n,const Sum&s){std::cout<<n<<" X="<<s.x<<" Y="<<s.y<<" mag="<<std::hypot(s.x,s.y)<<" valid/invalid="<<s.valid<<"/"<<s.invalid<<"\\n";};
 leg("A->B BASE",abf);leg("A->B +FB ",abfb);leg("B->A BASE",baf);leg("B->A +FB ",bafb);
 auto cl=[&](const char*n,const Sum&A,const Sum&B){double ma=std::hypot(A.x,A.y),cx=A.x+B.x,cy=A.y+B.y;std::cout<<n<<" closure="<<std::hypot(cx,cy)<<" = "<<100.0*std::hypot(cx,cy)/ma<<" % of |AB|; BA/AB="<<std::hypot(B.x,B.y)/ma<<"\\n";};cl("BASE",abf,baf);cl("+FB ",abfb,bafb);
 std::cout<<"\n===== B->A BASE vs +FB LOCALIZATION (1-second bins) =====\n";
 std::cout<<"sec base_dx base_dy fb_dx fb_dy diff_dx diff_dy diff_mag cumulative_diff_mag\n";
 double cumx=0,cumy=0;long long t0=m[b].cam;int lastsec=-1;Sum sb,sf;
 auto flush=[&](int sec){if(sec<0)return;double dx=sf.x-sb.x,dy=sf.y-sb.y;cumx+=dx;cumy+=dy;std::cout<<sec<<" "<<sb.x<<" "<<sb.y<<" "<<sf.x<<" "<<sf.y<<" "<<dx<<" "<<dy<<" "<<std::hypot(dx,dy)<<" "<<std::hypot(cumx,cumy)<<"\n";sb=Sum{};sf=Sum{};};
 for(int i=b+1;i<=c;i++){int sec=(int)((m[i].cam-t0)/1000000000LL);if(lastsec<0)lastsec=sec;if(sec!=lastsec){flush(lastsec);lastsec=sec;}double dt=(m[i].cam-m[i-1].cam)*1e-9;Step qb=estimate(im[i-1],im[i],dt,K,D,false),qf=estimate(im[i-1],im[i],dt,K,D,true);if(qb.valid){sb.x+=qb.du;sb.y+=qb.dv;sb.valid++;}else sb.invalid++;if(qf.valid){sf.x+=qf.du;sf.y+=qf.dv;sf.valid++;}else sf.invalid++;}
 flush(lastsec);
 std::cout<<"TOTAL FB-BASE: X="<<cumx<<" Y="<<cumy<<" mag="<<std::hypot(cumx,cumy)<<"\n";
 return 0;
}