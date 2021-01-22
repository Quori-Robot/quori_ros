#ifndef _QUORI_FACE_WINDOW_HPP_
#define _QUORI_FACE_WINDOW_HPP_

#include <cstdint>
#include <string>
#include <memory>
#include <functional>

#include "Monitor.hpp"

struct GLFWwindow;

namespace quori_face
{
  class WindowManager;

  class Window : public std::enable_shared_from_this<Window>
  {
  public:
    friend class WindowManager;

    typedef std::shared_ptr<Window> Ptr;
    typedef std::shared_ptr<const Window> ConstPtr;

    ~Window();

    void close();
    bool isClosed() const;

    static void pollEvents();
    
    std::uint32_t getWidth() const;
    std::uint32_t getHeight() const;

    void bind();
    void unbind();
    void swapBuffers();

    void execute(const std::function<void ()> &f);

    template<typename T>
    T execute(const std::function<T ()> &f)
    {
      std::uint8_t t[sizeof(T)];
      execute(std::bind([&]() {
        new (t) T(f());
      }));

      T &tt = *reinterpret_cast<T *>(t);
      T ret(tt);
      tt.~T();
      return ret;
    }

    void draw(const std::function<void ()> &f);

    template<typename... Args>
    static Ptr open(Args &&...args)
    {
      return Ptr(new Window(args...));
    }

  private:
    Window(const std::uint32_t width, const std::uint32_t height, const std::string &name);
    Window(const std::uint32_t width, const std::uint32_t height, const std::string &name, const Monitor::ConstPtr &monitor);

    GLFWwindow *getHandle();
    const GLFWwindow *getHandle() const;

    void close_impl();
    
    GLFWwindow *window_;
  };
}

#endif