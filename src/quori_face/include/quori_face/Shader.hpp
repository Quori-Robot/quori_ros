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

    /**
     * \fn compile
     * \brief Compile a shader from source
     * 
     * \param[in] type The shader type
     * \param[in] buffer The shader program source code
     * \param[in] length The buffer length in bytes
     * 
     * \return A Shader object
     */
    static Ptr compile(const Type type, const std::uint8_t *const buffer, const std::size_t length);
    
    /**
     * \fn compile
     * \brief Compile a shader from source
     * 
     * \param[in] type The shader type
     * \param[in] str The shader program source code
     * 
     * \return A Shader object
     */
    static Ptr compile(const Type type, const std::string &str);

    /**
     * \fn getHandle
     * \return The underlying OpenGL handle
     */
    std::uint32_t getHandle() const;

  private:
    Shader(const std::uint32_t handle);

    std::uint32_t handle_;
  };
}

#endif