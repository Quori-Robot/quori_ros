#ifndef _QUORI_FACE_SHADER_HPP_
#define _QUORI_FACE_SHADER_HPP_

#include <cstdint>
#include <memory>



namespace quori_face
{
  class Shader
  {
  public:
    enum class Type : std::uint8_t
    {
      Vertex,
      Fragment
    };

    typedef std::shared_ptr<Shader> Ptr;
    typedef std::shared_ptr<const Shader> ConstPtr;

    ~Shader();

    static Ptr compile(const Type type, const std::uint8_t *const buffer, const std::size_t length);
    static Ptr compile(const Type type, const std::string &str);

    std::uint32_t getHandle() const;

  private:
    Shader(const std::uint32_t handle);

    std::uint32_t handle_;
  };
}

#endif