#ifndef _QUORI_FACE_TRANSFORM_HPP_
#define _QUORI_FACE_TRANSFORM_HPP_

#include "Vector2.hpp"

namespace quori_face
{
  /**
   * \struct TransformStaticParameters
   * 
   * These parameters contain a physical description of the projector and head (used when computing transforms)
   * The names listed here directly correspond to the associated paper "LOW COST OPTICAL MECHANICAL SYSTEM
   * FOR HUMAN ROBOT INTERACTION" IMECE2018-87885
   */
  struct TransformStaticParameters
  {
    /**
     * Radius of the spherical head
     */
    double R;

    /**
     * Radius of the mirror
     */
    double r_m;

    /**
     * Radius of the base circle at the bottom of the head
     */
    double r_o;

    /**
     * Distance from the center of projection to the bottom of the head
     */
    double h;

    /**
     * The distance between the projection center and the intersection of the projection axis and the sphere
     */
    double L;

    /**
     * The angle between the axis of roation of the circle and the intersection of the projection axis and the sphere
     */
    double epsilon;

    /**
     * The parameter for calibrating the x and y-axis installation error
     */
    Vector2<double> delta;

    /**
     * The projector screen size in pixels
     */
    Vector2<std::uint32_t> screen_size;
  
    static const TransformStaticParameters DEFAULT;

    bool operator ==(const TransformStaticParameters &other) const;
    bool operator !=(const TransformStaticParameters &other) const;
  };

  /**
   * \struct SphericalCoordinate
   * 
   * These parameters contain a spherical coordinate used by the transform function
   */
  struct SphericalCoordinate
  {
    SphericalCoordinate();
    SphericalCoordinate(const double theta, const double psi);

    /**
     * Polar angle of spherical coordinates on the head
     */
    double theta;

    /**
     * Azimuthal angle of spherical coordinates on the head
     */
    double psi;

    SphericalCoordinate operator +(const SphericalCoordinate &rhs) const;
    SphericalCoordinate &operator +=(const SphericalCoordinate &rhs);

    static const SphericalCoordinate CENTER;
  };

  /**
   * \fn transform
   * Transforms a spherical coordinate into a pixel location
   * 
   * \param static_params The fixed parameters that have been calibrated for Quori's head
   * \param coord The spherical coordinate
   * 
   * \return A 2D pixel location
   */
  Vector2<double> transform(const TransformStaticParameters &static_params, const SphericalCoordinate &coord);

  
  float *generateLookupTable(const TransformStaticParameters &static_params, const SphericalCoordinate &min, const SphericalCoordinate &max, const Vector2<std::uint32_t> &size);
}

#endif