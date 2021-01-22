#ifndef _QUORI_FACE_CACHE_HPP_
#define _QUORI_FACE_CACHE_HPP_

#include <deque>
#include <cstdint>
#include <functional>
#include <mutex>

namespace quori_face
{
  template<typename K, typename V>
  class Cache
  {
  public:
    Cache(const std::size_t size = 5UL)
      : size_(size)
    {
    }

    const V &getOrCompute(const K &key, const std::function<V (const K &k)> &f)
    {
      std::lock_guard<std::mutex> guard(mut_);

      for (auto it = entries_.cbegin(); it != entries_.cend(); ++it)
      {
        if (it->first == key) return it->second;
      }
      
      entries_.push_front(std::make_pair(key, f(key)));
      if (entries_.size() > size_) entries_.pop_back();
      return entries_.front().second;
    }

  private:
    mutable std::mutex mut_;
    std::deque<std::pair<K, V>> entries_;
    std::size_t size_;
  };
}

#endif