#include "message.hpp"

using namespace quori;
using namespace quori::message;

Initialize::Initialize()
  : marker(static_cast<uint8_t>(Type::Initialize))
{
}

SetPositions::SetPositions()
  : marker(static_cast<uint8_t>(Type::SetPositions))
{
}

Set::Set()
  : marker(static_cast<uint8_t>(Type::Set))
{
}

SetPositionsRes::SetPositionsRes()
  : marker(static_cast<uint8_t>(Type::SetPositionsRes))
{
}

GetStates::GetStates()
  : marker(static_cast<uint8_t>(Type::GetStates))
{
}

SetVelocities::SetVelocities()
  : marker(static_cast<uint8_t>(Type::SetVelocities))
{
}

Coast::Coast()
  : marker(static_cast<uint8_t>(Type::Coast))
{
}

SetLimit::SetLimit()
  : marker(static_cast<uint8_t>(Type::SetLimit))
{
}

Log::Log()
  : marker(static_cast<uint8_t>(Type::Log))
{
  memset(message, 0, sizeof(message));
}

States::States()
  : marker(static_cast<uint8_t>(Type::States))
{
}

size_t message::string_length(const std::string &t)
{
  return t.size();
}

size_t message::string_length(const char *const t)
{
  return strlen(t);
}

const char *message::c_string(const std::string &t)
{
  return t.c_str();
}

const char *message::c_string(const char *const t)
{
  return t;
}