#pragma once

#include <ostream>
#include <memory>
#include <cstdint>

namespace quori_controller
{
  struct Joint
  {
    Joint();
    Joint(const std::string &name);

    typedef std::shared_ptr<const Joint> ConstPtr;
    typedef std::shared_ptr<Joint> Ptr;

    enum class Mode : std::uint8_t
    {
      Position,
      Velocity
    };

    std::string name;

    double command;
    double position;
    double velocity;
    double effort;

    Mode mode;
  };
}

std::ostream &operator <<(std::ostream &o, const quori_controller::Joint &joint);
std::ostream &operator <<(std::ostream &o, const quori_controller::Joint::Ptr &joint);
std::ostream &operator <<(std::ostream &o, const quori_controller::Joint::ConstPtr &joint);