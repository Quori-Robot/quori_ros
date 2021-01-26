#ifndef _QUORI_FACE_VECTOR2_HPP_
#define _QUORI_FACE_VECTOR2_HPP_

#include <cstdint>
#include <iostream>

namespace quori_face
{
  /**
   * \class Vector2
   * A 2D vector
   * 
   * \tparam T The element types
   */
  template<typename T>
  struct Vector2
  {
    /**
     * \fn Vector2
     * \brief Construct a zero'd Vector2
     */
    Vector2()
      : x(T())
      , y(T())
    {
    }

    /**
     * \fn Vector2
     */
    Vector2(const T x, const T y)
      : x(x)
      , y(y)
    {
    }
    
    T x;
    T y;

    bool operator ==(const Vector2<T> &other) const noexcept
    {
      return x == other.x && y == other.y;
    }

    bool operator !=(const Vector2<T> &other) const noexcept
    {
      return x != other.x || y != other.y;
    }

    Vector2<T> operator -(const Vector2<T> &rhs) const noexcept
    {
      return Vector2<T>(x - rhs.x, y - rhs.y);
    }

    Vector2<T> &operator -=(const Vector2<T> &rhs) noexcept
    {
      x -= rhs.x;
      y -= rhs.y;
      return *this;
    }

    Vector2<T> operator +(const Vector2<T> &rhs) const noexcept
    {
      return Vector2<T>(x + rhs.x, y + rhs.y);
    }

    Vector2<T> &operator +=(const Vector2<T> &rhs) noexcept
    {
      x += rhs.x;
      y += rhs.y;
      return *this;
    }

    Vector2<T> operator *(const Vector2<T> &rhs) noexcept
    {
      return Vector2<T>(x * rhs.x, y * rhs.y);
    }

    Vector2<T> &operator *=(const Vector2<T> &rhs) noexcept
    {
      x *= rhs.x;
      y *= rhs.y;
      return *this;
    }

    Vector2<T> operator /(const Vector2<T> &rhs) noexcept
    {
      return Vector2<T>(x / rhs.x, y / rhs.y);
    }

    Vector2<T> &operator /=(const Vector2<T> &rhs) noexcept
    {
      x /= rhs.x;
      y /= rhs.y;
      return *this;
    }
  };
}

template<typename T>
std::ostream &operator <<(std::ostream &o, const quori_face::Vector2<T> &value)
{
  return o << "<" << value.x << ", " << value.y << ">";
}

#endif