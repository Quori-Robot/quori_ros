#ifndef _QUORI_FACE_CACHE_HPP_
#define _QUORI_FACE_CACHE_HPP_

#include <deque>
#include <cstdint>
#include <functional>
#include <mutex>

namespace quori_face
{
  /**
   * \class Cache
   * \brief A simple memoization object that eliminates repetitive
   * computations.
   * 
   * \tparam K The key type
   * \tparam V The value type
   */
  template<typename K, typename V>
  class Cache
  {
  public:
    /**
     * \fn Cache
     * 
     * \param[in] size The maximum cache size
     */
    Cache(const std::size_t size = 5UL)
      : size_(size)
    {
    }

    /**
     * \fn getOrCompute
     * 
     * Get a value, either from cache, or by executing the function f.
     * 
     * \param[in] key The key to lookup
     * \param[in] f The function to execute to generate the value if not found in the cache
     */
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