#include "quori_face/Program.hpp"

#include "quori_face/gl.hpp"
#include <GL/gl3w.h>

#include <stdexcept>
#include <iostream>

#include "trace.hpp"


using namespace quori_face;

Program::~Program()
{
  QUORI_FACE_TRACE(glDeleteProgram(handle_));
}

Program::Ptr Program::link(const std::initializer_list<Shader::Ptr> &shaders)
{
  std::uint16_t handle = QUORI_FACE_TRACE(glCreateProgram());
  checkGlError();
  
  for (const Shader::Ptr &shader : shaders)
  {
    QUORI_FACE_TRACE(glAttachShader(handle, shader->getHandle()));
    checkGlError();
  }

  QUORI_FACE_TRACE(glLinkProgram(handle));
  checkGlError();

  GLint success = GL_TRUE;
  QUORI_FACE_TRACE(glGetProgramiv(handle, GL_LINK_STATUS, &success));
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

  return Ptr(new Program(handle));
}

std::uint32_t Program::getUniformLocation(const std::string &name) const
{
  return QUORI_FACE_TRACE(glGetUniformLocation(handle_, name.c_str()));
}

void Program::use()
{
  QUORI_FACE_TRACE(glUseProgram(handle_));
}

std::uint32_t Program::getHandle() const
{
  return handle_;
}

Program::Program(const std::uint16_t handle)
  : handle_(handle)
{
}