#include "apps/demo/map_yaml.hpp"
#include "apps/demo/map_io.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>

namespace hybrid_astar::demo {
namespace {
std::string trim(std::string s) {
  const auto first=s.find_first_not_of(" \t\r\n");
  if(first==std::string::npos)return {};
  const auto last=s.find_last_not_of(" \t\r\n");
  return s.substr(first,last-first+1);
}
std::string stripComment(const std::string& source) {
  char quote=0;
  for(std::size_t i=0;i<source.size();++i) {
    const char c=source[i];
    if(quote && c==quote){quote=0;continue;}
    if(!quote && (c=='\''||c=='"')){quote=c;continue;}
    if(!quote && c=='#')return source.substr(0,i);
  }
  if(quote)throw std::invalid_argument("Unterminated map YAML quote");
  return source;
}
std::string scalar(std::string value) {
  value=trim(value);
  if(value.size()>=2 && ((value.front()=='"'&&value.back()=='"')||
                         (value.front()=='\''&&value.back()=='\'')))
    value=value.substr(1,value.size()-2);
  if(value.empty()||value.find('\0')!=std::string::npos)
    throw std::invalid_argument("Empty or invalid map YAML value");
  return value;
}
double real(const std::string& s) {
  std::size_t parsed=0;
  double result=0;
  try {result=std::stod(trim(s),&parsed);}catch(const std::exception&) {
    throw std::invalid_argument("Invalid map YAML floating-point number");
  }
  if(parsed!=trim(s).size()||!std::isfinite(result))
    throw std::invalid_argument("Invalid map YAML floating-point number");
  return result;
}
Pose2D origin(std::string value) {
  value=trim(value);
  if(value.size()<5||value.front()!='['||value.back()!=']')
    throw std::invalid_argument("Map origin must be [x,y,yaw]");
  value=value.substr(1,value.size()-2);
  const auto comma1=value.find(',');
  const auto comma2=value.find(',',comma1==std::string::npos?0:comma1+1);
  if(comma1==std::string::npos||comma2==std::string::npos||
     value.find(',',comma2+1)!=std::string::npos)
    throw std::invalid_argument("Map origin must contain exactly three coordinates");
  return {real(value.substr(0,comma1)),real(value.substr(comma1+1,comma2-comma1-1)),
          real(value.substr(comma2+1))};
}
}
RosMapMetadata parseRosMapYaml(std::istream& input) {
  std::map<std::string,std::string> values;
  std::size_t total=0,lines=0;
  std::string line;
  while(std::getline(input,line)) {
    total+=line.size();++lines;
    if(total>65536||lines>512||line.size()>2048)
      throw std::invalid_argument("Map YAML exceeds size limits");
    line=trim(stripComment(line));
    if(line.empty()||line=="---"||line=="...")continue;
    const auto sep=line.find(':');
    if(sep==std::string::npos)
      throw std::invalid_argument("Expected flat key: value map YAML");
    const auto key=trim(line.substr(0,sep));
    if(key!="image"&&key!="resolution"&&key!="origin"&&key!="negate"&&
       key!="occupied_thresh"&&key!="free_thresh"&&key!="mode")
      throw std::invalid_argument("Unsupported map YAML field: "+key);
    if(!values.emplace(key,scalar(line.substr(sep+1))).second)
      throw std::invalid_argument("Duplicate map YAML field: "+key);
  }
  if(!values.contains("image")||!values.contains("resolution")||!values.contains("origin"))
    throw std::invalid_argument("Map YAML requires image, resolution and origin");
  RosMapMetadata m;
  m.image=values.at("image");
  if(m.image.size()>1024||m.image.find('\n')!=std::string::npos)
    throw std::invalid_argument("Invalid image path in map YAML");
  m.resolutionMeters=real(values.at("resolution"));
  m.origin=origin(values.at("origin"));
  if(values.contains("negate")) {
    const auto& n=values.at("negate");
    if(n!="0"&&n!="1")
      throw std::invalid_argument("Map negate must be 0 or 1");
    m.negate=n=="1";
  }
  if(values.contains("occupied_thresh"))
    m.occupiedThreshold=real(values.at("occupied_thresh"));
  if(values.contains("free_thresh"))m.freeThreshold=real(values.at("free_thresh"));
  if(values.contains("mode")&&values.at("mode")!="trinary")
    throw std::invalid_argument("Only ROS trinary map mode is supported");
  if(m.resolutionMeters<=0||m.resolutionMeters>2||
     std::abs(m.origin.x)>10000||std::abs(m.origin.y)>10000||std::abs(m.origin.yaw)>10000||
     m.freeThreshold<0||m.occupiedThreshold>1||m.freeThreshold>=m.occupiedThreshold)
    throw std::invalid_argument("Map YAML resolution, origin or thresholds out of bounds");
  return m;
}
std::shared_ptr<ackermann::MapSnapshot> loadRosMapYaml(const std::string& filename) {
  std::ifstream in(filename,std::ios::binary);
  if(!in)throw std::invalid_argument("Cannot open map YAML");
  const auto m=parseRosMapYaml(in);
  const auto image=std::filesystem::path(m.image);
  const auto base=std::filesystem::absolute(std::filesystem::path(filename)).parent_path();
  const auto target=image.is_absolute()?image:base/image;
  auto snapshot=loadPgm(target.lexically_normal().string(),m.resolutionMeters,
                        PgmOccupancyOptions{m.occupiedThreshold,m.freeThreshold,m.negate,true});
  snapshot->origin=m.origin;
  snapshot->sourceId="ros-map-yaml";
  return snapshot;
}
} // namespace hybrid_astar::demo
