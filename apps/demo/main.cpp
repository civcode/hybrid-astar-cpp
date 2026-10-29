#include "apps/demo/demo.hpp"
#include "apps/demo/map_io.hpp"
#include "apps/demo/map_yaml.hpp"
#include <algorithm>
#include <charconv>
#include <cstdint>
#include <limits>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#ifdef __unix__
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif
#ifndef HYBRID_ASTAR_DEMO_WEB_ROOT
#define HYBRID_ASTAR_DEMO_WEB_ROOT "apps/demo/web"
#endif

namespace {
using hybrid_astar::Pose2D;
using hybrid_astar::demo::Options;
std::string percentDecode(std::string_view value) {
  std::string result;
  for(std::size_t i=0;i<value.size();++i) {
    if(value[i]=='+')result+=' ';
    else if(value[i]=='%') {
      if(i+2>=value.size())throw std::invalid_argument("Incomplete URL escape");
      unsigned int byte=0;
      auto p=value.substr(i+1,2);
      const auto [end,ec]=std::from_chars(p.data(),p.data()+p.size(),byte,16);
      if(ec!=std::errc{}||end!=p.data()+p.size()||byte<32||byte==127)
        throw std::invalid_argument("Invalid URL escape");
      result+=static_cast<char>(byte);i+=2;
    } else if(static_cast<unsigned char>(value[i])<32)throw std::invalid_argument("Control in URL");
    else result+=value[i];
  }
  return result;
}
std::map<std::string,std::string> queryParameters(std::string_view query) {
  std::map<std::string,std::string> values;
  while(!query.empty()) {
    const auto amp=query.find('&');
    auto segment=query.substr(0,amp);
    const auto eq=segment.find('=');
    const auto key=percentDecode(segment.substr(0,eq));
    const auto value=eq==std::string_view::npos?std::string{}:percentDecode(segment.substr(eq+1));
    if(key.empty()||!values.emplace(key,value).second)
      throw std::invalid_argument("Duplicate or empty query parameter");
    if(amp==std::string_view::npos)break;
    query.remove_prefix(amp+1);
  }
  return values;
}
double number(const std::string& value) {
  std::size_t n=0;const double v=std::stod(value,&n);
  if(n!=value.size()||!std::isfinite(v)||std::abs(v)>10000)
    throw std::invalid_argument("Invalid numeric coordinate");
  return v;
}
bool flag(const std::string& value) {
  if(value=="1"||value=="true")return true;
  if(value=="0"||value=="false")return false;
  throw std::invalid_argument("Boolean option must be 0 or 1");
}
// Search work budget in expanded nodes. The demo is a single-threaded
// browser service; enforce the same 1M hard ceiling as the library default.
std::size_t budgetValue(const std::string& value) {
  std::size_t budget=0;
  const auto [end,error]=std::from_chars(value.data(),value.data()+value.size(),budget);
  if(value.empty()||error!=std::errc{}||end!=value.data()+value.size()||
     budget<100||budget>1000000)
    throw std::invalid_argument("Budget must be an integer between 100 and 1000000 expansions");
  return budget;
}
// Limit this UI experiment to 8, 12 or 16 acceptances for all models.
// The attempt and collision-check ceilings remain specific to each model.
std::size_t acceptedReplacementLimit(const std::string& value) {
  if(value=="8")return 8;
  if(value=="12")return 12;
  if(value=="16")return 16;
  throw std::invalid_argument("Accepted replacements must be 8, 12 or 16");
}
Options requestOptions(const std::map<std::string,std::string>& params) {
  Options o;
  if(auto it=params.find("edits");it!=params.end())o.edits=hybrid_astar::demo::parseCellEdits(it->second);
  if(auto it=params.find("editRevision");it!=params.end()) {
    std::uint64_t value=0;
    const auto& raw=it->second;
    const auto [end,error]=std::from_chars(raw.data(),raw.data()+raw.size(),value);
    if(raw.empty()||error!=std::errc{}||end!=raw.data()+raw.size()||value>1000000000ULL)
      throw std::invalid_argument("Invalid editRevision");
    o.editGeneration=value;
  }
  if(auto it=params.find("scenario");it!=params.end())o.scenario=it->second;
  if(auto it=params.find("model");it!=params.end())o.model=it->second;
  if(auto it=params.find("refine");it!=params.end())o.refine=flag(it->second);
  if(auto it=params.find("maxAccepted");it!=params.end())
    o.maxAcceptedReplacements=acceptedReplacementLimit(it->second);
  if(auto it=params.find("curved");it!=params.end())o.curvedRefinement=flag(it->second);
  if(auto it=params.find("steeringSmooth");it!=params.end())
    o.steeringSmoothness=flag(it->second);
  if(auto it=params.find("steeringWeight");it!=params.end())
    o.steeringWeightMetersSquared=number(it->second);
  if(auto it=params.find("performance");it!=params.end())o.performanceHeuristic=flag(it->second);
  if(auto it=params.find("rsDistance");it!=params.end())o.rsDistanceHeuristic=flag(it->second);
  if(auto it=params.find("improve");it!=params.end())o.improve=flag(it->second);
  if(auto it=params.find("ompl");it!=params.end())o.ompl=flag(it->second);
  if(auto it=params.find("budget");it!=params.end())o.maxExpansions=budgetValue(it->second);
  if(auto it=params.find("inflate");it!=params.end())o.footprintInflationEnabled=flag(it->second);
  if(auto it=params.find("inflationMeters");it!=params.end()) {
    const double value=number(it->second);
    if(value<0||value>10)
      throw std::invalid_argument("Footprint inflation must be between 0 and 10 m");
    o.footprintInflationMeters=value;
  }
  auto coords=[&](std::string_view prefix)->std::optional<Pose2D> {
    const std::string key(prefix);
    const auto x=params.find(key+'x'),y=params.find(key+'y'),heading=params.find(key+"yaw");
    if(x==params.end()&&y==params.end()&&heading==params.end())return {};
    if(x==params.end()||y==params.end()||heading==params.end())
      throw std::invalid_argument("Supply x, y and yaw together");
    return Pose2D{number(x->second),number(y->second),number(heading->second)*std::acos(-1.0)/180.};
  };
  o.start=coords("start");o.goal=coords("goal");
  for(const auto& [name,_]:params) {
    if(name!="scenario"&&name!="model"&&name!="edits"&&name!="editRevision"&&name!="refine"&&name!="maxAccepted"&&name!="curved"&&name!="steeringSmooth"&&name!="steeringWeight"&&name!="performance"&&name!="rsDistance"&&
       name!="improve"&&name!="ompl"&&name!="budget"&&name!="inflate"&&
       name!="inflationMeters"&&name!="startx"&&name!="starty"&&
       name!="startyaw"&&name!="goalx"&&name!="goaly"&&name!="goalyaw")
      throw std::invalid_argument("Unknown option: "+name);
  }
  return o;
}
std::string jsonError(std::string_view e) {
  std::string value="{\"error\":\"";
  for(char c:e) {
    if(c=='"'||c=='\\')value+='\\';
    if(static_cast<unsigned char>(c)<32)continue;
    value+=c;
  }
  return value+"\"}";
}
std::string loadAsset(const std::string& name) {
  std::ifstream stream(std::string(HYBRID_ASTAR_DEMO_WEB_ROOT)+"/"+name,std::ios::binary);
  if(!stream)throw std::runtime_error("Cannot load demo asset: "+name);
  std::ostringstream buffer;buffer<<stream.rdbuf();return buffer.str();
}
#ifdef __unix__
void sendReply(int fd,int code,std::string_view type,const std::string& body) {
  std::ostringstream header;
  header<<"HTTP/1.1 "<<code<<(code==200?" OK":code==400?" Bad Request":code==404?" Not Found":" Method Not Allowed")<<"\r\n"
        <<"Content-Type: "<<type<<"\r\nContent-Length: "<<body.size()
        <<"\r\nConnection: close\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\n"
        <<"Content-Security-Policy: default-src 'self'; style-src 'self'; script-src 'self'; connect-src 'self'\r\n\r\n";
  const auto payload=header.str()+body;
  for(std::size_t offset=0;offset<payload.size();) {
    auto sent=::send(fd,payload.data()+offset,payload.size()-offset,MSG_NOSIGNAL);
    if(sent<=0)break;
    offset+=static_cast<std::size_t>(sent);
  }
}
void handle(int client,const std::shared_ptr<const hybrid_astar::ackermann::MapSnapshot>& customMap) {
  // Bound both headers and form data. A timeout prevents a half-open local
  // connection from blocking this deliberately single-threaded demo server.
  timeval timeout{5,0};
  ::setsockopt(client,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
  ::setsockopt(client,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
  constexpr std::size_t maxHeader=8192,maxBody=65536;
  std::string request;
  request.reserve(maxHeader);
  char chunk[4096];
  std::size_t endHeaders=std::string::npos;
  while((endHeaders=request.find("\r\n\r\n"))==std::string::npos) {
    if(request.size()>maxHeader) {
      sendReply(client,400,"application/json",jsonError("Headers too long"));return;
    }
    const auto n=::recv(client,chunk,sizeof(chunk),0);
    if(n<=0) {
      sendReply(client,400,"application/json",jsonError("Incomplete HTTP headers"));return;
    }
    request.append(chunk,static_cast<std::size_t>(n));
    if(request.size()>maxHeader+maxBody) {
      sendReply(client,400,"application/json",jsonError("Request too long"));return;
    }
  }
  if(endHeaders>maxHeader) {
    sendReply(client,400,"application/json",jsonError("Headers too long"));return;
  }
  const auto lineEnd=request.find("\r\n");
  if(lineEnd==std::string::npos) {
    sendReply(client,400,"application/json",jsonError("Invalid request line"));return;
  }
  const auto line=std::string_view(request).substr(0,lineEnd);
  const auto methodEnd=line.find(' ');
  const auto urlEnd=methodEnd==std::string_view::npos?std::string_view::npos:line.find(' ',methodEnd+1);
  if(urlEnd==std::string_view::npos||urlEnd-methodEnd>2050) {
    sendReply(client,400,"application/json",jsonError("Invalid request URL"));return;
  }
  const auto method=line.substr(0,methodEnd);
  const auto url=line.substr(methodEnd+1,urlEnd-methodEnd-1);
  const auto queryAt=url.find('?');
  const std::string route(url.substr(0,queryAt)); // remains valid while POST body grows
  if(method!="GET"&&method!="POST") {
    sendReply(client,405,"application/json",jsonError("Only GET and planning POST are supported"));return;
  }
  if(method=="POST"&&route!="/api/plan") {
    sendReply(client,405,"application/json",jsonError("POST is only supported for /api/plan"));return;
  }
  try {
    std::string_view params;
    if(method=="POST") {
      bool sawLength=false,sawType=false;
      std::size_t length=0;
      auto header=std::string_view(request).substr(lineEnd+2,endHeaders-lineEnd-2);
      while(!header.empty()) {
        const auto split=header.find("\r\n");
        const auto part=header.substr(0,split);
        const auto colon=part.find(':');
        if(colon==std::string_view::npos)throw std::invalid_argument("Invalid HTTP header");
        std::string key(part.substr(0,colon));
        for(char& ch:key) if(ch>='A'&&ch<='Z')ch=char(ch-'A'+'a');
        const auto start=part.find_first_not_of(" \t",colon+1);
        const auto value=start==std::string_view::npos?std::string_view{}:part.substr(start);
        if(key=="content-length") {
          if(sawLength)throw std::invalid_argument("Duplicate Content-Length");
          sawLength=true;
          const auto [stop,err]=std::from_chars(value.data(),value.data()+value.size(),length);
          if(err!=std::errc{}||stop!=value.data()+value.size()||length>maxBody)
            throw std::invalid_argument("Invalid or oversized Content-Length");
        } else if(key=="content-type") {
          if(sawType)throw std::invalid_argument("Duplicate Content-Type");
          sawType=true;
          if(!value.starts_with("application/x-www-form-urlencoded"))
            throw std::invalid_argument("Expected URL-encoded planning form");
        } else if(key=="transfer-encoding")
          throw std::invalid_argument("Chunked transfer is unsupported");
        if(split==std::string_view::npos)break;
        header.remove_prefix(split+2);
      }
      if(!sawLength||!sawType)throw std::invalid_argument("POST requires Content-Length and Content-Type");
      const std::size_t offset=endHeaders+4;
      if(request.size()-offset>length)throw std::invalid_argument("Excess POST body bytes");
      while(request.size()-offset<length) {
        const auto remaining=length-(request.size()-offset);
        const auto n=::recv(client,chunk,std::min(remaining,sizeof(chunk)),0);
        if(n<=0)throw std::invalid_argument("Truncated planning form");
        request.append(chunk,static_cast<std::size_t>(n));
      }
      params=std::string_view(request).substr(offset,length);
    } else params=queryAt==std::string_view::npos?std::string_view{}:url.substr(queryAt+1);
    if(route=="/"||route=="/index.html")sendReply(client,200,"text/html; charset=utf-8",loadAsset("index.html"));
    else if(route=="/map_geometry.js")sendReply(client,200,"text/javascript; charset=utf-8",loadAsset("map_geometry.js"));
    else if(route=="/map_editor.js")sendReply(client,200,"text/javascript; charset=utf-8",loadAsset("map_editor.js"));
    else if(route=="/app.js")sendReply(client,200,"text/javascript; charset=utf-8",loadAsset("app.js"));
    else if(route=="/style.css")sendReply(client,200,"text/css; charset=utf-8",loadAsset("style.css"));
    else if(route=="/api/capabilities")sendReply(client,200,"application/json",hybrid_astar::demo::capabilitiesJson(customMap.get()));
    else if(route=="/api/plan") {
      auto options=requestOptions(queryParameters(params));
      options.customMap=customMap;
      sendReply(client,200,"application/json",hybrid_astar::demo::toJson(hybrid_astar::demo::run(options)));
    } else sendReply(client,404,"application/json",jsonError("Not found"));
  }catch(const std::exception& e) {
    sendReply(client,400,"application/json",jsonError(e.what()));
  }
}
void serve(unsigned int port,bool allowContainerInterface,const std::shared_ptr<const hybrid_astar::ackermann::MapSnapshot>& customMap) {
  const int server=::socket(AF_INET,SOCK_STREAM,0);
  if(server<0)throw std::runtime_error("Failed to create socket");
  const int reuse=1;::setsockopt(server,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse));
  sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_port=htons(static_cast<std::uint16_t>(port));
  addr.sin_addr.s_addr=htonl(allowContainerInterface?INADDR_ANY:INADDR_LOOPBACK);
  if(::bind(server,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))!=0 || ::listen(server,16)!=0) {
    ::close(server);throw std::runtime_error("Cannot bind demo localhost port");
  }
  std::cout<<"Hybrid A* demo: http://127.0.0.1:"<<port<<"\nPress Ctrl-C to stop.\n";
  if(allowContainerInterface) std::cerr<<"WARNING: listening on all container interfaces; publish Docker ports on 127.0.0.1 only.\n";
  std::cout<<std::flush;
  for(;;) {
    const int client=::accept(server,nullptr,nullptr);
    if(client>=0){handle(client,customMap);::close(client);}
  }
}
#endif
void usage(){std::cout<<"Usage: hybrid_astar_demo [--model ackermann|differential|mecanum] [--port 8765] [--bind 127.0.0.1|0.0.0.0] [--once --scenario open|reverse|parking|warehouse|blocked]"
  " [--start x y yawDeg] [--goal x y yawDeg] [--map FILE.pgm --resolution METERS | --map-yaml FILE.yaml] [--output FILE.json --once]"
  " [--ompl] [--rs-distance] [--refine] [--curved] [--steering-smoothness] [--steering-weight (0,100]] [--max-accepted 8|12|16] [--performance|--certified] [--inflate METERS] [--improve] [--budget 100..1000000]\n"
  "Default: obstacle-grid heuristic (weight 1.0); --certified selects Euclidean. --once prints JSON.\n";}
}
int main(int argc,char** argv) {
  try {
    Options options;unsigned int port=8765;bool once=false;bool allowContainerInterface=false;
    std::string mapFilename,mapYamlFilename,outputFilename;double mapResolution=.25;
    bool mapResolutionSpecified=false;
    for(int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      if(arg=="--help"||arg=="-h"){usage();return 0;}
      if(arg=="--once")once=true;
      else if(arg=="--port"){
        if(i+1>=argc)throw std::invalid_argument("Missing port");
        const std::string raw=argv[++i];
        std::size_t parsed=0;const auto n=std::stoul(raw,&parsed);
        if(parsed!=raw.size()||n<1024||n>65535)
          throw std::invalid_argument("Port must be 1024..65535");
        port=static_cast<unsigned int>(n);
      } else if(arg=="--bind") {
        if(i+1>=argc)throw std::invalid_argument("Missing bind address");
        const std::string bind=argv[++i];
        if(bind=="127.0.0.1")allowContainerInterface=false;
        else if(bind=="0.0.0.0")allowContainerInterface=true;
        else throw std::invalid_argument("Bind address must be 127.0.0.1 or 0.0.0.0");
      } else if(arg=="--map") {
        if(i+1>=argc)throw std::invalid_argument("Missing PGM filename");
        mapFilename=argv[++i];options.scenario="custom";
      } else if(arg=="--map-yaml") {
        if(i+1>=argc)throw std::invalid_argument("Missing map YAML filename");
        mapYamlFilename=argv[++i];options.scenario="custom";
      } else if(arg=="--resolution") {
        if(i+1>=argc)throw std::invalid_argument("Missing map resolution");
        mapResolution=number(argv[++i]);mapResolutionSpecified=true;
      } else if(arg=="--output") {
        if(i+1>=argc)throw std::invalid_argument("Missing output filename");
        outputFilename=argv[++i];
      } else if(arg=="--model") {
        if(i+1>=argc)throw std::invalid_argument("Missing kinematic model");
        options.model=argv[++i];
      } else if(arg=="--scenario") {
        if(i+1>=argc)throw std::invalid_argument("Missing scenario");
        options.scenario=argv[++i];
      } else if(arg=="--start"||arg=="--goal") {
        if(i+3>=argc)throw std::invalid_argument("Pose requires x y yawDeg");
        Pose2D pose{number(argv[++i]),number(argv[++i]),number(argv[++i])*std::acos(-1.0)/180.};
        (arg=="--start"?options.start:options.goal)=pose;
      } else if(arg=="--ompl")options.ompl=true;
      else if(arg=="--rs-distance")options.rsDistanceHeuristic=true;
      else if(arg=="--refine")options.refine=true;
      else if(arg=="--max-accepted"){
        if(i+1>=argc)throw std::invalid_argument("Missing accepted-replacement limit");
        options.maxAcceptedReplacements=acceptedReplacementLimit(argv[++i]);
      }
      else if(arg=="--curved")options.curvedRefinement=true;
      else if(arg=="--steering-smoothness")options.steeringSmoothness=true;
      else if(arg=="--steering-weight"){
        if(i+1>=argc)throw std::invalid_argument("Missing steering smoothness weight");
        options.steeringWeightMetersSquared=number(argv[++i]);
        options.steeringSmoothness=true;
      }
      else if(arg=="--performance")options.performanceHeuristic=true;
      else if(arg=="--certified")options.performanceHeuristic=false;
      else if(arg=="--improve")options.improve=true;
      else if(arg=="--inflate"){
        if(i+1>=argc)throw std::invalid_argument("Missing inflation meters");
        const double meters=number(argv[++i]);
        if(meters<0||meters>10)
          throw std::invalid_argument("Footprint inflation must be between 0 and 10 m");
        options.footprintInflationEnabled=true;
        options.footprintInflationMeters=meters;
      }
      else if(arg=="--budget"){
        if(i+1>=argc)throw std::invalid_argument("Missing budget");
        options.maxExpansions=budgetValue(argv[++i]);
      }
      else throw std::invalid_argument("Unknown flag: "+arg);
    }
    if(!mapFilename.empty()||!mapYamlFilename.empty())options.scenario="custom";
    if(!mapFilename.empty()&&!mapYamlFilename.empty())
      throw std::invalid_argument("--map and --map-yaml are mutually exclusive");
    if(!mapYamlFilename.empty()) {
      if(mapResolutionSpecified)
        throw std::invalid_argument("YAML resolution comes from the map file; omit --resolution");
      options.customMap=hybrid_astar::demo::loadRosMapYaml(mapYamlFilename);
    } else if(!mapFilename.empty())
      options.customMap=hybrid_astar::demo::loadPgm(mapFilename,mapResolution);
    if(!outputFilename.empty()&&!once)throw std::invalid_argument("--output requires --once");
    if(!once && options.model!="ackermann")
      throw std::invalid_argument("--model is a --once option; select the model in the browser for server mode");
    if(once) {
      const auto json=hybrid_astar::demo::toJson(hybrid_astar::demo::run(options));
      if(outputFilename.empty())std::cout<<json<<'\n';
      else {
        std::ofstream output(outputFilename,std::ios::binary|std::ios::trunc);
        if(!output||!(output<<json<<'\n'))throw std::runtime_error("Cannot write output JSON");
      }
      return 0;
    }
#ifdef __unix__
    serve(port,allowContainerInterface,options.customMap);
#else
    throw std::runtime_error("Local demo HTTP server requires POSIX sockets; use --once on this platform");
#endif
  } catch(const std::exception& e) {
    std::cerr<<"Demo error: "<<e.what()<<'\n';return 2;
  }
}
