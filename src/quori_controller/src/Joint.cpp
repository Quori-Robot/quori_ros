#include "Joint.hpp"

using namespace quori_controller;

std::ostream &operator <<(std::ostream &o, const Joint::Mode &mode)
{
  switch (mode)
  {
    case Joint::Mode::Position: return o << "Position";
    case Joint::Mode::Velocity: return o << "Velocity";
  }

  return o;
}

Joint::Joint()
  : command(double())
  , position(double())
  , velocity(double())
  , effort(double())
  , mode(Mode::Position)
{
}

Joint::Joint(const std::string &name)
  : name(name)
  , command(double())
  , position(double())
  , velocity(double())
  , effort(double())
  , mode(Mode::Position)
{

}


std::ostream &operator <<(std::ostream &o, const Joint &joint)
{
  return o
    << joint.name
    << "(cmd = " << joint.command
    << ", pos = " << joint.position
    << ", vel = " << joint.velocity
    << ", eff = " << joint.effort
    << ", mode = " << joint.mode
    << ")";
}

std::ostream &operator <<(std::ostream &o, const Joint::Ptr &joint)
{
  return o
    << joint->name
    << "(cmd = " << joint->command
    << ", pos = " << joint->position
    << ", vel = " << joint->velocity
    << ", eff = " << joint->effort
    << ", mode = " << joint->mode
    << ")";
}

std::ostream &operator <<(std::ostream &o, const Joint::ConstPtr &joint)
{
  return o
    << joint->name
    << "(cmd = " << joint->command
    << ", pos = " << joint->position
    << ", vel = " << joint->velocity
    << ", eff = " << joint->effort
    << ", mode = " << joint->mode
    << ")";
}
