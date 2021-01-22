#include "quori_face/ElementBufferObject.hpp"

using namespace quori_face;

ElementBufferObject::ElementBufferObject()
{

}

ElementBufferObject::~ElementBufferObject()
{

}

std::uint32_t ElementBufferObject::getHandle() const noexcept
{
  return handle_;
} 