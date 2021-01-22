#ifndef _QUORI_FACE_MONITOR_HPP_
#define _QUORI_FACE_MONITOR_HPP_

#include <memory>
#include <vector>

struct GLFWmonitor;

namespace quori_face
{
  class Monitor
  {
  public:
    typedef std::shared_ptr<Monitor> Ptr;
    typedef std::shared_ptr<const Monitor> ConstPtr;

    ~Monitor();

    static std::vector<Monitor::Ptr> getMonitors();
    static Monitor::Ptr getPrimaryMonitor();


    bool isPrimary() const;
    std::string getName() const;

    GLFWmonitor *getHandle() const;

  private:
    Monitor(GLFWmonitor *const handle);

    GLFWmonitor *handle_;
  };
}

#endif