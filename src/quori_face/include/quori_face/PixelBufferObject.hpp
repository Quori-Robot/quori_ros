#ifndef _QUORI_FACE_PIXEL_BUFFER_OBJECT_HPP_
#define _QUORI_FACE_PIXEL_BUFFER_OBJECT_HPP_

#include <cstdint>

namespace quori_face
{
  class PixelBufferObject
  {
  public:
    PixelBufferObject(const std::size_t size);
    ~PixelBufferObject();

    void bind(const std::size_t i);
    void unbind();
    std::uint32_t getHandle(const std::size_t i);

  private:
    std::uint32_t handles_[2];
  };
}

#endif