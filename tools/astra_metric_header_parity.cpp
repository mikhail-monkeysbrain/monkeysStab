#include "../src/astra_metric.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct Row{int interval; double r0,p0,y0,r1,p1,y1,range0; float ax,ay,bx,by;};
static std::vector<std::string> split(const std::string&s){std::vector<std::string>v;std::stringstream q(s);std::string x;while(std::getline(q,x,','))v.push_back(x);return v;}

int main(int ac,char**av){
  if(ac!=2){std::cerr<<"usage: astra_metric_header_parity metric_inputs.csv\n";return 2;}
  std::ifstream f(av[1]); if(!f){std::cerr<<"cannot open input\n";return 2;}
  std::string s; std::getline(f,s); std::vector<Row> all;
  while(std::getline(f,s)){
    auto v=split(s); if(v.size()!=12) continue;
    all.push_back({std::stoi(v[0]),std::stod(v[1]),std::stod(v[2]),std::stod(v[3]),std::stod(v[4]),std::stod(v[5]),std::stod(v[6]),std::stod(v[7]),std::stof(v[8]),std::stof(v[9]),std::stof(v[10]),std::stof(v[11])});
  }
  astra_metric::V3 total(0,0,0); int intervals=0;
  for(size_t i=0;i<all.size();){
    size_t j=i+1; while(j<all.size()&&all[j].interval==all[i].interval) ++j;
    std::vector<cv::Point2f> a,b; a.reserve(j-i); b.reserve(j-i);
    for(size_t k=i;k<j;k++){a.emplace_back(all[k].ax,all[k].ay);b.emplace_back(all[k].bx,all[k].by);}
    const auto R0=astra_metric::rotation(all[i].r0,all[i].p0,all[i].y0);
    const auto R1=astra_metric::rotation(all[i].r1,all[i].p1,all[i].y1);
    const auto d=astra_metric::metric(a,b,R0,R1,all[i].range0);
    if(!std::isfinite(d[0])){std::cerr<<"bad interval "<<all[i].interval<<"\n";return 3;}
    total+=d; ++intervals; i=j;
  }
  std::cout<<std::fixed<<std::setprecision(9)
           <<"ASTRA REUSABLE HEADER PARITY\n"
           <<"intervals="<<intervals<<"\n"
           <<"N/E = ("<<total[0]*1000<<", "<<total[1]*1000<<") mm\n"
           <<"magnitude = "<<std::hypot(total[0],total[1])*1000<<" mm\n";
}
