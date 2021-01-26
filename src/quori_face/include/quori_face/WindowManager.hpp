#ifndef _QUORI_FACE_WINDOW_MANAGER_HPP_
#define _QUORI_FACE_WINDOW_MANAGER_HPP_

#include "Window.hpp"

#include <vector>

namespace quori_face
{
  /**
   * \class WindowManager
   * 
   * A singleton that manages window events
   */
  class WindowManager
  {
  public:
    typedef std::vector<Window::Ptr> WindowList;
    
    ~WindowManager();

    /**
     * \fn attach
     * 
     * Attaches a window to the WindowManager
     * \param window The window to attach
     */
    void attach(const Window::Ptr &window);

    /**
     * \fn detach
     * 
     * Detaches a window from the WindowManager
     * \param window The window to detach
     */
    void detach(const Window::Ptr &window);

    /**
     * \fn lookupByHandle
     * Given a GLFW handle, return a pointer to the window object that's attached to the WindowManager.
     * 
     * \param handle The handle to look up
     */
    Window::Ptr lookupByHandle(const GLFWwindow *const handle) const;

    /**
     * \fn begin
     * Iterate over the windows attached to the WindowManager
     * 
     * \return An iterator to the beginning of the window list.
     */
    WindowList::iterator begin();
    
    /**
     * \fn end
     * \return An iterator to the end of the window list.
     */
    WindowList::iterator end();

    /**
     * \fn cbegin
     * Iterate over the windows attached to the WindowManager
     * 
     * \return A const iterator to the beginning of the window list.
     */
    WindowList::const_iterator cbegin() const;
    
    /**
     * \fn cend
     * \return A const iterator to the end of the window list.
     */
    WindowList::const_iterator cend() const;

    /**
     * \fn ref
     * \return The WindowManager singleton
     */
    static WindowManager &ref();

    /**
     * \fn close
     * 
     * Close a given window
     * \param window The window to close
     */
    void close(const Window::Ptr &window);

  private:
    WindowManager();

    WindowList windows_;
  };
}

#endif