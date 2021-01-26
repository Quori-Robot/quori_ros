#ifndef _QUORI_FACE_GL_HPP_
#define _QUORI_FACE_GL_HPP_

namespace quori_face
{
  /**
   * \fn checkGlError
   * \brief Throws a `std::runtime_error` if the OpenGL error flag is set
   */
  void checkGlError();
}

#endif