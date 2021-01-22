#include "quori_face/Shader.hpp"

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#include <cstring>
#include <unordered_map>
#include <iostream>


#include "quori_face/gl.hpp"

#include "trace.hpp"

using namespace quori_face;

namespace
{
  const static std::unordered_map<std::uint8_t, GLenum> TYPE_MAP {
    { static_cast<std::uint8_t>(Shader::Type::Vertex), GL_VERTEX_SHADER },
    { static_cast<std::uint8_t>(Shader::Type::Fragment), GL_FRAGMENT_SHADER },
  };
}

Shader::~Shader()
{
  QUORI_FACE_TRACE(glDeleteShader(handle_));
}

Shader::Ptr Shader::compile(const Shader::Type type, const std::uint8_t *const buffer, const std::size_t len)
{
  const auto it = TYPE_MAP.find(static_cast<std::uint8_t>(type));
  if (it == TYPE_MAP.cend())
  {
    throw std::runtime_error("Unknown shader type");
  }

  const std::uint32_t handle = QUORI_FACE_TRACE(glCreateShader(it->second));
  checkGlError();

  const GLchar *string[1] = { reinterpret_cast<const GLchar *>(buffer) };
  const GLint length[1] = { (GLint)len };

  QUORI_FACE_TRACE(glShaderSource(handle, 1, string, length));
  checkGlError();

  QUORI_FACE_TRACE(glCompileShader(handle));
  checkGlError();

  GLint success = GL_TRUE;

  QUORI_FACE_TRACE(glGetShaderiv(handle, GL_COMPILE_STATUS, &success));
  checkGlError();
  if (success == GL_FALSE)
  {
    GLint length = 0;
    QUORI_FACE_TRACE(glGetProgramiv(handle, GL_INFO_LOG_LENGTH, &length));
    char *buffer = new char[length];
    QUORI_FACE_TRACE(glGetProgramInfoLog(handle, length, NULL, buffer));
    const std::string message(buffer, buffer + length);
    delete[] buffer;
    throw std::runtime_error(message);
  }

  return Ptr(new Shader(handle));
}

Shader::Ptr Shader::compile(const Type type, const std::string &str)
{
  return compile(type, reinterpret_cast<const std::uint8_t *>(str.data()), str.size());
}

std::uint32_t Shader::getHandle() const
{
  return handle_;
}


Shader::Shader(const std::uint32_t handle)
  : handle_(handle)
{

}