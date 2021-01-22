#ifndef _QUORI_FACE_ELEMENT_BUFFER_OBJECT_HPP_
#define _QUORI_FACE_ELEMENT_BUFFER_OBJECT_HPP_

#include <cstdint>

namespace quori_face
{
  class ElementBufferObject
  {
  public:
    ElementBufferObject();
    ~ElementBufferObject();

    std::uint32_t getHandle() const noexcept; 

  private:
    std::uint32_t handle_;
  };
}

#endif