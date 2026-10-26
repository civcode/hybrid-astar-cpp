#include "apps/demo/map_edits.hpp"
#include <charconv>
#include <limits>
#include <stdexcept>
#include <string_view>
namespace hybrid_astar::demo {
namespace {
std::uint64_t uintValue(std::string_view token) {
  if(token.empty())throw std::invalid_argument("Empty edit integer");
  std::uint64_t value=0;
  const auto [end,err]=std::from_chars(token.data(),token.data()+token.size(),value);
  if(err!=std::errc{}||end!=token.data()+token.size())
    throw std::invalid_argument("Invalid cell edit integer");
  return value;
}
}
std::vector<CellEdit> parseCellEdits(std::string_view encoded) {
  if(encoded.size()>65536)throw std::invalid_argument("Cell edit payload too large");
  std::vector<CellEdit> edits;
  while(!encoded.empty()) {
    if(edits.size()>=4096)throw std::invalid_argument("Too many edited cells (maximum 4096)");
    const auto comma=encoded.find(',');
    const auto entry=encoded.substr(0,comma);
    const auto colon=entry.find(':');
    if(colon==std::string_view::npos || entry.find(':',colon+1)!=std::string_view::npos)
      throw std::invalid_argument("Expected cell edit index:value");
    const auto index=uintValue(entry.substr(0,colon));
    if(index>=262144)throw std::invalid_argument("Cell edit index exceeds map limits");
    const auto value=entry.substr(colon+1);
    std::int8_t occupancy;
    if(value=="100")occupancy=100;
    else if(value=="0")occupancy=0;
    else if(value=="-1")occupancy=-1;
    else throw std::invalid_argument("Cell edit must be blocked (100), free (0) or unknown (-1)");
    if(!edits.empty()&&index<=edits.back().index)
      throw std::invalid_argument("Cell edits must be unique and sorted");
    edits.push_back({static_cast<std::size_t>(index),occupancy});
    if(comma==std::string_view::npos)break;
    encoded.remove_prefix(comma+1);
    if(encoded.empty())throw std::invalid_argument("Trailing cell edit delimiter");
  }
  return edits;
}
std::shared_ptr<const ackermann::MapSnapshot> applyCellEdits(
    const std::shared_ptr<const ackermann::MapSnapshot>& base,
    const std::vector<CellEdit>& edits, std::uint64_t generation) {
  if(!base)throw std::invalid_argument("Map edits require a valid base map");
  if(generation==0) {
    if(!edits.empty())throw std::invalid_argument("Nonempty cell edits require a new revision");
    return base;
  }
  if(generation>1000000000ULL ||
     base->revision>std::numeric_limits<std::uint64_t>::max()-generation)
    throw std::invalid_argument("Map edit revision outside supported range");
  if(edits.size()>4096)throw std::invalid_argument("Too many cell edits");
  auto result=std::make_shared<ackermann::MapSnapshot>(*base);
  std::size_t previous=0;
  bool first=true;
  for(const auto& edit:edits) {
    if(edit.index>=result->cells.size() || (!first&&edit.index<=previous) ||
       (edit.occupancy!=0&&edit.occupancy!=100&&edit.occupancy!=-1))
      throw std::invalid_argument("Invalid map cell edit");
    result->cells[edit.index]=edit.occupancy;
    first=false;previous=edit.index;
  }
  result->revision=base->revision+generation;
  return result;
}
} // namespace hybrid_astar::demo
