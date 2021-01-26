#include "quori_face/gl.hpp"
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#include <GL/glu.h>

#include <stdexcept>
#include <iostream>

#include "trace.hpp"

using namespace quori_face;

namespace
{
  bool is_inited(false);

  __attribute__((constructor))
  bool init()
  {
    // FIXME: This isn't thread-safe
    if (is_inited) return true;

    if (!QUORI_FACE_TRACE(glfwInit()))
    {
      std::cerr << "GLFW initialization failed" << std::endl;
      exit(1);
      return false;
    }

    QUORI_FACE_TRACE(glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3));
    QUORI_FACE_TRACE(glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2));
    QUORI_FACE_TRACE(glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE));
    
    is_inited = true;
    return true;
  }

  __attribute__((destructor))
  void deinit()
  {
    if (!is_inited) return;

    QUORI_FACE_TRACE(glfwTerminate());
  }
}

void quori_face::checkGlError()
{
  const GLenum error = glGetError();
  if (error == GL_NO_ERROR) return;
  throw std::runtime_error(reinterpret_cast<const char *>(gluErrorString(error)));
}