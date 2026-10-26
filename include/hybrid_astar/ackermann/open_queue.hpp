#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <optional>
#include <queue>
#include <vector>

namespace hybrid_astar::ackermann {
// The binary lazy queue preserves the original baseline implementation.
// Indexed variants keep at most one entry per currently active search label.
enum class OpenQueueMode { BinaryLazy, FourAryLazy, IndexedBinary, IndexedFourAry };
struct OpenEntry { double f, h; std::size_t node; };
struct OpenEntryWorse {
  bool operator()(const OpenEntry& a, const OpenEntry& b) const noexcept {
    if (a.f!=b.f) return a.f>b.f;
    if (a.h!=b.h) return a.h>b.h;
    return a.node>b.node;
  }
};
// std::priority_queue deliberately remains the baseline implementation.
// Exposing its vector capacity lets diagnostics account for reserved storage.
class BinaryOpenHeap : public std::priority_queue<OpenEntry,std::vector<OpenEntry>,OpenEntryWorse> {
public:
  std::size_t capacity() const noexcept {return this->c.capacity();}
};
class SearchOpenQueue {
public:
  explicit SearchOpenQueue(OpenQueueMode mode):mode_(mode) {}
  bool empty() const noexcept { return indexedOrFourAry()?heap_.empty():binary_.empty(); }
  std::size_t size() const noexcept { return indexedOrFourAry()?heap_.size():binary_.size(); }
  std::size_t storageBytes() const noexcept {
    return (indexedOrFourAry()?heap_.capacity():binary_.capacity())*sizeof(OpenEntry)
         +location_.capacity()*sizeof(std::size_t);
  }
  // If oldNode still resides in an indexed heap, replace it atomically.
  // Popped/expanded old nodes are absent and get a new heap insertion.
  // Old-node hints are ignored by the lazy queue variants.
  bool push(OpenEntry entry, std::optional<std::size_t> oldNode=std::nullopt) {
    if(!indexedOrFourAry()) {binary_.push(entry);return false;}
    if(isIndexed()) {
      if(entry.node>=location_.size())location_.resize(entry.node+1,missing);
      if(oldNode && *oldNode<location_.size()&&location_[*oldNode]!=missing) {
        const auto index=location_[*oldNode];
        location_[*oldNode]=missing;
        heap_[index]=entry;
        location_[entry.node]=index;
        repair(index);
        return true;
      }
    }
    heap_.push_back(entry);
    if(isIndexed())location_[entry.node]=heap_.size()-1;
    siftUp(heap_.size()-1);
    return false;
  }
  OpenEntry pop() {
    if(!indexedOrFourAry()) {
      const auto item=binary_.top();binary_.pop();return item;
    }
    const auto item=heap_.front();
    if(isIndexed())location_[item.node]=missing;
    if(heap_.size()==1) {heap_.pop_back();return item;}
    heap_.front()=heap_.back();heap_.pop_back();
    if(isIndexed())location_[heap_.front().node]=0;
    siftDown(0);
    return item;
  }
private:
  static constexpr auto missing=std::numeric_limits<std::size_t>::max();
  bool isIndexed() const noexcept {
    return mode_==OpenQueueMode::IndexedBinary||mode_==OpenQueueMode::IndexedFourAry;
  }
  bool indexedOrFourAry() const noexcept {return mode_!=OpenQueueMode::BinaryLazy;}
  std::size_t arity() const noexcept {
    return mode_==OpenQueueMode::FourAryLazy||mode_==OpenQueueMode::IndexedFourAry?4:2;
  }
  bool better(const OpenEntry& a,const OpenEntry& b) const noexcept {
    return OpenEntryWorse{}(b,a);
  }
  void swapEntries(std::size_t a,std::size_t b) noexcept {
    std::swap(heap_[a],heap_[b]);
    if(isIndexed()) {location_[heap_[a].node]=a;location_[heap_[b].node]=b;}
  }
  void siftUp(std::size_t index) noexcept {
    const auto base=arity();
    while(index) {
      const auto parent=(index-1)/base;
      if(!better(heap_[index],heap_[parent]))break;
      swapEntries(index,parent);index=parent;
    }
  }
  void siftDown(std::size_t index) noexcept {
    const auto base=arity();
    while(index<(heap_.size()-1)/base+1) {
      const auto first=index*base+1;
      if(first>=heap_.size())break;
      auto best=index;
      const auto stop=std::min(heap_.size(),first+base);
      for(auto child=first;child<stop;++child)
        if(better(heap_[child],heap_[best]))best=child;
      if(best==index)break;
      swapEntries(index,best);index=best;
    }
  }
  void repair(std::size_t index) noexcept {
    if(index && better(heap_[index],heap_[(index-1)/arity()]))siftUp(index);
    else siftDown(index);
  }
  OpenQueueMode mode_;
  BinaryOpenHeap binary_;
  std::vector<OpenEntry> heap_;
  std::vector<std::size_t> location_;
};
} // namespace hybrid_astar::ackermann
