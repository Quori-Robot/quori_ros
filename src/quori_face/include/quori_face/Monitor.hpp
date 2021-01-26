#ifndef _QUORI_FACE_MONITOR_HPP_
#define _QUORI_FACE_MONITOR_HPP_

#include <memory>
#include <vector>

struct GLFWmonitor;

namespace quori_face
{
  /**
   * \class Monitor
   * 
   * Represents a display connected to the computer
   */
  class Monitor
  {
  public:
    typedef std::shared_ptr<Monitor> Ptr;
    typedef std::shared_ptr<const Monitor> ConstPtr;

    ~Monitor();

    /**
     * \fn getMonitors
     * \return A list of the monitors connected to the computer
     */
    static std::vector<Monitor::Ptr> getMonitors();

    /**
     * \fn getPrimaryMonitor
     * \return The primary monitor connected to the computer
     */
    static Monitor::Ptr getPrimaryMonitor();

    /**
     * \fn isPrimary
     * \return true if the monitor is the primary one, false otherwise
     */
    bool isPrimary() const;

    /**
     * \fn getName
     * \return The OS-defined name of the monitor
     */
    std::string getName() const;

    /**
     * \fn getHandle
     * \return The underlying GLFW window pointer
     */
    GLFWmonitor *getHandle() const;

  private:
    Monitor(GLFWmonitor *const handle);

    GLFWmonitor *handle_;
  };
}

#endif