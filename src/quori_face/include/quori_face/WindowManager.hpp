#ifndef _QUORI_FACE_WINDOW_MANAGER_HPP_
#define _QUORI_FACE_WINDOW_MANAGER_HPP_

#include "Window.hpp"

#include <vector>

namespace quori_face
{
  class WindowManager
  {
  public:
    typedef std::vector<Window::Ptr> WindowList;
    
    ~WindowManager();

    void attach(const Window::Ptr &window);
    void detach(const Window::Ptr &window);

    Window::Ptr lookupByHandle(const GLFWwindow *const handle) const;

    WindowList::iterator begin();
    WindowList::iterator end();

    WindowList::const_iterator cbegin() const;
    WindowList::const_iterator cend() const;

    static WindowManager &ref();

    void close(const Window::Ptr &window);

  private:
    WindowManager();

    WindowList windows_;
  };
}

#endif