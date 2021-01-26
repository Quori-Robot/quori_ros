#ifndef _QUORI_FACE_PIXEL_BUFFER_OBJECT_HPP_
#define _QUORI_FACE_PIXEL_BUFFER_OBJECT_HPP_

#include <cstdint>

namespace quori_face
{
  /**
   * \class PixelBufferObject
   * 
   * A pipelined pixel buffer object (composed to 2 pixel buffer handles)
   */
  class PixelBufferObject
  {
  public:

    PixelBufferObject(const std::size_t size);
    ~PixelBufferObject();

    /**
     * \fn bind
     * Binds a pixel buffer object to the current OpenGL context.
     * 
     * \param i The index of the pixel buffer object to bind (either 0 or 1)
     */
    void bind(const std::size_t i);

    /**
     * \fn unbind
     * Unbind the current pixel buffer object from the current OpenGL context.
     */
    void unbind();

    /**
     * \fn getHandle
     * \param i The index of the pixel buffer object to get
     * \return The underlying pixel buffer OpenGL handle
     */
    std::uint32_t getHandle(const std::size_t i);

  private:
    std::uint32_t handles_[2];
  };
}

#endif