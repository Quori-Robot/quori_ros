#include "quori_face/WindowManager.hpp"
#include <GLFW/glfw3.h>

#include <unordered_map>

#include "trace.hpp"

using namespace quori_face;

namespace
{
  void onWindowClose(GLFWwindow *handle)
  {
    const auto window = WindowManager::ref().lookupByHandle(handle);
    if (!window) return;
    window->close();
  }

  void onWindowResize(GLFWwindow* handle, int width, int height)
  {
    const auto window = WindowManager::ref().lookupByHandle(handle);
    if (!window) return;
    glViewport(0, 0, width, height);
  }
}

WindowManager::WindowManager()
{

}

WindowManager::~WindowManager()
{

}

void WindowManager::attach(const Window::Ptr &window)
{
  for (auto it = windows_.begin(); it != windows_.end(); ++it)
  {
    if (*it == window) return;
  }

  QUORI_FACE_TRACE(glfwSetWindowCloseCallback(window->getHandle(), &onWindowClose));
  QUORI_FACE_TRACE(glfwSetFramebufferSizeCallback(window->getHandle(), &onWindowResize));
  windows_.push_back(window);
}

void WindowManager::detach(const Window::Ptr &window)
{
  for (auto it = windows_.begin(); it != windows_.end();)
  {
    if (*it == window)
    {
      QUORI_FACE_TRACE(glfwSetWindowCloseCallback((*it)->getHandle(), nullptr));
      QUORI_FACE_TRACE(glfwSetFramebufferSizeCallback((*it)->getHandle(), nullptr));
      it = windows_.erase(it);
      continue;
    }
    
    ++it;
  }
}

Window::Ptr WindowManager::lookupByHandle(const GLFWwindow *const handle) const
{
  for (auto it = windows_.begin(); it != windows_.end(); ++it)
  {
    if ((*it)->getHandle() == handle) return *it;
  }

  return nullptr;
}

WindowManager::WindowList::iterator WindowManager::begin()
{
  return windows_.begin();
}

WindowManager::WindowList::iterator WindowManager::end()
{
  return windows_.end();
}

WindowManager::WindowList::const_iterator WindowManager::cbegin() const
{
  return windows_.cbegin();
}

WindowManager::WindowList::const_iterator WindowManager::cend() const
{
  return windows_.cend();
}

WindowManager &WindowManager::ref()
{
  static WindowManager instance;
  return instance;
}