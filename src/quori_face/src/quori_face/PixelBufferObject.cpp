#include "quori_face/PixelBufferObject.hpp"

#include <GL/gl3w.h>

#include "trace.hpp"

using namespace quori_face;

PixelBufferObject::PixelBufferObject(const std::size_t size)
{
  QUORI_FACE_TRACE(glGenBuffers(2, handles_));
  QUORI_FACE_TRACE(glBindBuffer(GL_PIXEL_UNPACK_BUFFER, handles_[0]));
  QUORI_FACE_TRACE(glBufferData(GL_PIXEL_UNPACK_BUFFER, size, 0, GL_STREAM_DRAW));
  QUORI_FACE_TRACE(glBindBuffer(GL_PIXEL_UNPACK_BUFFER, handles_[1]));
  QUORI_FACE_TRACE(glBufferData(GL_PIXEL_UNPACK_BUFFER, size, 0, GL_STREAM_DRAW));
  QUORI_FACE_TRACE(glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0));
}

PixelBufferObject::~PixelBufferObject()
{
  QUORI_FACE_TRACE(glDeleteBuffers(2, handles_));
}

void PixelBufferObject::bind(const std::size_t i)
{
  QUORI_FACE_TRACE(glBindBuffer(GL_PIXEL_UNPACK_BUFFER, handles_[i % 2UL]));
} 

void PixelBufferObject::unbind()
{
  QUORI_FACE_TRACE(glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0));
}


std::uint32_t PixelBufferObject::getHandle(const std::size_t i)
{
  return handles_[i % 2UL];
}
