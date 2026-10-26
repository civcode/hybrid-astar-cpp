#include "apps/demo/map_io.hpp"
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
namespace hybrid_astar::demo {
namespace {
std::string token(std::istream& in) {
  std::string t; char c;
  while(in.get(c)) {
    if(c=='#') { in.ignore(std::numeric_limits<std::streamsize>::max(),'\n');continue; }
    if(c==' '||c=='\t'||c=='\r'||c=='\n')continue;
    t+=c;break;
  }
  if(t.empty())throw std::invalid_argument("Truncated PGM header or pixel data");
  while(in.get(c)) {
    if(c==' '||c=='\t'||c=='\n'||c=='\r') {
      if(c=='\r'&&in.peek()=='\n')in.get();
      break;
    }
    if(c=='#')throw std::invalid_argument("PGM comment must follow whitespace");
    if(t.size()>=24)throw std::invalid_argument("Oversized PGM numeric token");
    t+=c;
  }
  return t;
}
unsigned parseUInt(std::string s) {
  if(s.empty())throw std::invalid_argument("Empty PGM integer");
  unsigned v=0;
  for(char c:s) {
    if(c<'0'||c>'9'||v>(10000000u-unsigned(c-'0'))/10u)
      throw std::invalid_argument("Invalid PGM integer");
    v=v*10+unsigned(c-'0');
  }
  return v;
}
}
std::shared_ptr<ackermann::MapSnapshot> parsePgm(std::istream& in,double resolution,PgmOccupancyOptions options) {
  if(!std::isfinite(options.occupiedThreshold)||!std::isfinite(options.freeThreshold)||
     options.freeThreshold<0||options.occupiedThreshold>1||
     options.freeThreshold>=options.occupiedThreshold)
    throw std::invalid_argument("Invalid PGM occupancy thresholds");
  if(!std::isfinite(resolution)||resolution<=0||resolution>2)
    throw std::invalid_argument("PGM resolution must be finite and in (0,2] m/pixel");
  const auto magic=token(in);
  if(magic!="P2"&&magic!="P5")throw std::invalid_argument("Only P2/P5 PGM is supported");
  const unsigned width=parseUInt(token(in)),height=parseUInt(token(in)),maxValue=parseUInt(token(in));
  if(width<1||height<1||width>1024||height>1024||
     std::size_t(width)*height>262144||maxValue<1||maxValue>255)
    throw std::invalid_argument("PGM dimensions or maxval out of supported bounds");
  auto map=std::make_shared<ackermann::MapSnapshot>();
  map->width=width;map->height=height;map->resolutionMeters=resolution;
  map->frameId="demo/map";map->sourceId="imported-pgm";map->revision=1;
  map->cells.resize(std::size_t(width)*height);
  for(std::size_t row=0;row<height;++row)for(std::size_t col=0;col<width;++col) {
    unsigned pixel;
    if(magic=="P2")pixel=parseUInt(token(in));
    else {
      const auto value=in.get();
      if(value==std::char_traits<char>::eof())
        throw std::invalid_argument("Truncated binary PGM raster");
      pixel=unsigned(value);
    }
    if(pixel>maxValue)throw std::invalid_argument("PGM pixel exceeds maxval");
    std::int8_t occupancy;
    if(options.rosTrinary) {
      const double probability=options.negate?double(pixel)/maxValue:1.-double(pixel)/maxValue;
      occupancy=probability>options.occupiedThreshold?100:
                probability<options.freeThreshold?0:-1;
    } else {
      occupancy=pixel*2u>=maxValue?0:100;
      if(maxValue==255 && pixel==205)occupancy=-1;
    }
    map->cells[(height-1-row)*width+col]=occupancy;
  }
  if(magic=="P5") {
    if(in.peek()!=std::char_traits<char>::eof())
      throw std::invalid_argument("Unexpected trailing binary PGM data");
  } else {
    // Comments and whitespace after the text raster are valid.
    while(in.peek()!=std::char_traits<char>::eof()) {
      const char c=char(in.peek());
      if(c==' '||c=='\t'||c=='\n'||c=='\r')in.get();
      else if(c=='#')in.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
      else throw std::invalid_argument("Unexpected trailing PGM pixels");
    }
  }
  return map;
}
std::shared_ptr<ackermann::MapSnapshot> loadPgm(const std::string& filename,double resolution,PgmOccupancyOptions options) {
  std::ifstream file(filename,std::ios::binary);
  if(!file)throw std::invalid_argument("Unable to open PGM file");
  return parsePgm(file,resolution,options);
}
}
