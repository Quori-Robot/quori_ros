#include "quori_face/Window.hpp"
#include "quori_face/WindowManager.hpp"

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#include <stdexcept>
#include <atomic>

#include "trace.hpp"

using namespace quori_face;



void throwGlfwError()
{
  throw std::runtime_error("GLFW Error");
}

Window::Window(const std::uint32_t width, const std::uint32_t height, const std::string &name)
{
  window_ = QUORI_FACE_TRACE(glfwCreateWindow(width, height, name.c_str(), nullptr, nullptr));
  if (!window_) throwGlfwError();

  glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

  execute([] {
    QUORI_FACE_TRACE(gl3wInit());
  });
}

Window::Window(const std::uint32_t width, const std::uint32_t height, const std::string &name, const Monitor::ConstPtr &monitor)
{
  window_ = QUORI_FACE_TRACE(glfwCreateWindow(width, height, name.c_str(), monitor->getHandle(), nullptr));
  if (!window_) throwGlfwError();

  glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);


  execute([] {
    QUORI_FACE_TRACE(gl3wInit());
  });
}

Window::~Window()
{
  if (!window_) return;
  QUORI_FACE_TRACE(glfwDestroyWindow(window_));
}

void Window::close()
{
  WindowManager::ref().detach(shared_from_this());
  close_impl();
}

bool Window::isClosed() const
{
  return window_ == nullptr;
}

void Window::pollEvents()
{
  QUORI_FACE_TRACE(glfwPollEvents());
}

std::uint32_t Window::getWidth() const
{
  if (!window_) throw std::runtime_error("Window is closed");
  
  int width = 0;
  QUORI_FACE_TRACE(glfwGetWindowSize(window_, &width, nullptr));
  return width;
}

std::uint32_t Window::getHeight() const
{
  if (!window_) throw std::runtime_error("Window is closed");

  int height = 0;
  QUORI_FACE_TRACE(glfwGetWindowSize(window_, nullptr, &height));
  return height;
}

GLFWwindow *Window::getHandle()
{
  return window_;
}

const GLFWwindow *Window::getHandle() const
{
  return window_;
}

void Window::bind()
{
  QUORI_FACE_TRACE(glfwMakeContextCurrent(window_));
}

void Window::unbind()
{
  QUORI_FACE_TRACE(glfwMakeContextCurrent(nullptr));
}

void Window::swapBuffers()
{
  QUORI_FACE_TRACE(glfwSwapBuffers(window_));
}

void Window::execute(const std::function<void ()> &f)
{
  if (!window_) throw std::runtime_error("Window is closed");
  
  bind();
  f();
  unbind();
}

void Window::draw(const std::function<void ()> &f)
{
  execute([this, &f]() {
    f();
    swapBuffers();
  });
}

void Window::close_impl()
{
  if (!window_) return;
  QUORI_FACE_TRACE(glfwDestroyWindow(window_));
  window_ = nullptr;
}