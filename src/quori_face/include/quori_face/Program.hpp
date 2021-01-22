#ifndef _QUORI_FACE_PROGRAM_HPP_
#define _QUORI_FACE_PROGRAM_HPP_

#include <cstdint>
#include <memory>

#include "Shader.hpp"

namespace quori_face
{
  class Program
  {
  public:
    typedef std::shared_ptr<Program> Ptr;
    typedef std::shared_ptr<const Program> ConstPtr;

    ~Program();

    static Ptr link(const std::initializer_list<Shader::Ptr> &shaders);

    std::uint32_t getUniformLocation(const std::string &name) const;

    void use();

    std::uint32_t getHandle() const;

  private:
    Program(const std::uint16_t handle);

    std::uint32_t handle_;
  };
}

#endif