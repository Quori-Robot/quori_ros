#ifndef _QUORI_MESSAGE_HPP_
#define _QUORI_MESSAGE_HPP_

#include <stdint.h>
#include <string>
#include <string.h>

#define QUORI_MESSAGE __attribute__((packed))

namespace quori
{
  namespace message
  {
    enum class Type : uint8_t
    {
      Initialize = 0x1,
      SetPositions = 0x2,
      GetStates = 0x3,
      Set = 0x4,
      SetVelocities,
      Coast,
      SetLimit,
      SetPositionsRes = 0xD0,
      Initialized = 0xF0,
      States = 0xF1,
      Log = 0xF2
    };

    struct QUORI_MESSAGE Initialize
    {
      Initialize();
      uint8_t marker;
    };

    struct QUORI_MESSAGE SetPositions
    {
      SetPositions();
      uint8_t marker;
      float positions[3];
    };

    struct QUORI_MESSAGE Set
    {
      Set();
      uint8_t marker;
      float positions[3];
      float velocities[3];
      float efforts[3];
    };

    struct QUORI_MESSAGE SetPositionsRes
    {
      SetPositionsRes();
      uint8_t marker;
      float values[4];
    };

    struct QUORI_MESSAGE GetStates
    {
      GetStates();
      uint8_t marker;
    };

    struct QUORI_MESSAGE SetVelocities
    {
      SetVelocities();
      uint8_t marker;
      float velocities[3];
    };

    struct QUORI_MESSAGE Coast
    {
      Coast();
      uint8_t marker;
    };

    struct QUORI_MESSAGE SetLimit
    {
      SetLimit();
      uint8_t marker;
      bool limit;
    };

    size_t string_length(const std::string &t);
    size_t string_length(const char *const t);

    const char *c_string(const std::string &t);
    const char *c_string(const char *const t);

    struct QUORI_MESSAGE Initialized
    {
      uint8_t marker;
      char _0[16];
      char _1[16];
      char _2[16];
      uint8_t modes[3];

    private:
      Initialized() = delete;
    };

    struct QUORI_MESSAGE Log
    {
      Log();
      uint8_t marker;
      char message[32];
    };

    struct QUORI_MESSAGE States
    {
      States();

      uint8_t marker;
      uint8_t measured_presence;
      uint8_t positions_presence;

      float measured[3];
      float positions[3];
      // float velocities[3];
      // float efforts[3];
    };
  }
  
}

#endif