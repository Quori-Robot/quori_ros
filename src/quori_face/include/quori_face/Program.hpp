#ifndef _QUORI_FACE_PROGRAM_HPP_
#define _QUORI_FACE_PROGRAM_HPP_

#include <cstdint>
#include <memory>

#include "Shader.hpp"

namespace quori_face
{
  /**
   * \class Program
   * Represents a shader pipeline program
   */
  class Program
  {
  public:
    typedef std::shared_ptr<Program> Ptr;
    typedef std::shared_ptr<const Program> ConstPtr;

    ~Program();

    /**
     * \fn link
     * Construct a program with the given shader pipeline.
     * \param[in] shaders The shader pipeline, in order
     */
    static Ptr link(const std::initializer_list<Shader::Ptr> &shaders);

    /**
     * \fn getUniformLocation
     */
    std::uint32_t getUniformLocation(const std::string &name) const;

    /**
     * \fn use
     * 
     * Bind the program to the current OpenGL context
     */
    void use();

    /**
     * \fn getHandle
     * \return The underlying OpenGL handle
     */
    std::uint32_t getHandle() const;

  private:
    Program(const std::uint16_t handle);

    std::uint32_t handle_;
  };
}

#endif