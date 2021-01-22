#include "quori_face/Texture.hpp"

#include <GL/gl3w.h>

#include "trace.hpp"

using namespace quori_face;

Texture::~Texture()
{
  glDeleteTextures(1, &handle_);
}

Texture::Ptr Texture::create(std::size_t rows, std::size_t cols, float *const data)
{
  std::uint32_t handle;
  QUORI_FACE_TRACE(glGenTextures(1, &handle));

  QUORI_FACE_TRACE(glBindTexture(GL_TEXTURE_2D, handle));

  QUORI_FACE_TRACE(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
  QUORI_FACE_TRACE(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

  QUORI_FACE_TRACE(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
  QUORI_FACE_TRACE(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

  QUORI_FACE_TRACE(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, cols, rows, 0, GL_RGB, GL_FLOAT, data));
  QUORI_FACE_TRACE(glGenerateMipmap(GL_TEXTURE_2D));

  return Ptr(new Texture(handle));
}

Texture::Ptr Texture::create(std::size_t rows, std::size_t cols, const std::uint32_t format, const std::uint8_t *const data)
{
  std::uint32_t handle;
  QUORI_FACE_TRACE(glGenTextures(1, &handle));

  QUORI_FACE_TRACE(glBindTexture(GL_TEXTURE_2D, handle));

  QUORI_FACE_TRACE(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
  QUORI_FACE_TRACE(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

  QUORI_FACE_TRACE(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
  QUORI_FACE_TRACE(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

  QUORI_FACE_TRACE(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, cols, rows, 0, format, GL_UNSIGNED_BYTE, data));
  QUORI_FACE_TRACE(glGenerateMipmap(GL_TEXTURE_2D));

  return Ptr(new Texture(handle));
}


void Texture::bind()
{
  QUORI_FACE_TRACE(glBindTexture(GL_TEXTURE_2D, handle_));
}

std::uint32_t Texture::getHandle() const
{
  return handle_;
}


Texture::Texture(const std::uint32_t handle)
  : handle_(handle)
{
}