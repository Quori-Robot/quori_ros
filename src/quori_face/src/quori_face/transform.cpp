// This code was largely copied from an early implementation by UPenn. The author
// does not know what the hard-coded values below mean.

#include "quori_face/transform.hpp"
#include "quori_face/Cache.hpp"

#include <cmath>
#include <complex>
#include <thread>
#include <memory>
#include <vector>

using namespace quori_face;

namespace
{
  template<typename T>
  inline T square(const T value)
  {
    return value * value;
  }

  template<typename T>
  inline T clamp(const T min, const T value, const T max)
  {
    if (value < min) return min;
    if (value > max) return max;
    return value;
  }

  // Closed form solution to the transform
  // parameters unknown
  double mTot(const double m, const double d, const double rm, const double R, const double del)
  {
    const double m2 = square(m);
    const double m3 = m2 * m;
    const double m5 = m3 * m2;
    const double d2 = square(d);
    const double d3 = d2 * d;
    const double rm2 = square(rm);
    const double rm4 = square(rm2);
    const double R2 = square(R);
    const double R3 = R2 * R;
    const double R4 = R3 * R;
    const double del2 = square(del);
    const double del3 = del2 * del;
    const double del4 = del3 * del;
    const double m2p1 = m2 + 1.0;
    const double m2p12 = square(m2p1);
    const double m2p13 = m2p12 * m2p1;
    const double Rddel = R + d - del;
    const double Rddel2 = square(Rddel);
    const double Rddelrm = Rddel + rm;
    const double para1 = R * rm - 2.0 * R * del - 2.0 * d * del + d * rm - 2.0 * del * rm + 2.0 * del2 + 4.0 * R2 * m2 + 2.0 * del2 * m2 + d * m2 * rm - 2.0 * del * m2 * rm + 4.0 * R * d * m2 - 6.0 * R * del * m2 + 3.0 * R * m2 * rm - 2.0 * d * del * m2;
    const double para2 = (8 * R * del3 + 8.0 * d * del3 - 4.0 * del4 - 4.0 * R2 * del2 - 16.0 * R4 * m2 + R2 * rm2 - 4.0 * d2 * del2 - 4.0 * del4 * m2 + d2 * rm2 + 4.0 * del2 * rm2 - 32.0 * R3 * d * m2 + 24 * R * del3 * m2 + 48.0 * R3 * del * m2 + 8.0 * d * del3 * m2 - 16.0 * R2 * d2 * m2 - 52.0 * R2 * del2 * m2 + 9.0 * R2 * m2 * rm2 - 4.0 * d2 * del2 * m2 + d2 * m2 * rm2 + 4.0 * del2 * m2 * rm2 - 8.0 * R * d * del2 + 2.0 * R * d * rm2 - 4.0 * R * del * rm2 - 4.0 * d * del * rm2 - 40 * R * d * del2 * m2 + 16 * R * d2 * del * m2 + 64.0 * R2 * d * del * m2 + 6.0 * R * d * m2 * rm2 - 12.0 * R * del * m2 * rm2 - 4.0 * d * del * m2 * rm2);
    const double para3 = 2.0 * R * d - 2.0 * R * del - 2.0 * d * del + R2 + d2 - 3.0 * R2 * m2 + d2 * m2 + 2.0 * R * d * m2 + 2.0 * R * del * m2 - 2.0 * d * del * m2;
    const double para4 = 18.0 * R4 * m * rm2 - 36.0 * R4 * m3 * rm2 - 54.0 * R4 * m5 * rm2 + 54.0 * R2 * d2 * m * rm2 + 36 * R * d3 * m3 * rm2 + 36.0 * R3 * d * m3 * rm2 + 18 * R * d3 * m5 * rm2 - 18.0 * R3 * d * m5 * rm2 + 36.0 * R2 * del2 * m * rm2 + 36.0 * R3 * del * m3 * rm2 + 90.0 * R3 * del * m5 * rm2 + 108.0 * R2 * d2 * m3 * rm2 + 54.0 * R2 * d2 * m5 * rm2 - 36.0 * R2 * del2 * m5 * rm2 + 18 * R * d3 * m * rm2 + 54.0 * R3 * d * m * rm2 - 54.0 * R3 * del * m * rm2 + 36 * R * d * del2 * m * rm2 - 54 * R * d2 * del * m * rm2 - 108.0 * R2 * d * del * m * rm2 + 72 * R * d * del2 * m3 * rm2 - 108.0 * R * d2 * del * m3 * rm2 - 144.0 * R2 * d * del * m3 * rm2 + 36.0 * R * d * del2 * m5 * rm2 - 54 * R * d2 * del * m5 * rm2 - 36.0 * R2 * d * del * m5 * rm2;
    const std::complex<double> para10 = (m2p13 * std::pow(para2, 3) + 108.0 * R2 * m2 * rm4 * m2p12 * Rddel2 * square(para3));
    const std::complex<double> para11 = std::sqrt(para10);
    const std::complex<double> para5 = std::pow((1.7320508075688 * para11 + para4), (1.0 / 3.0));
    const std::complex<double> para6 = 4330209751607469.0 * para5;
    const double para7 = 2.49809726143892e16 * m2p1 * para2;
    const double para8 = 9007199254740992 * R * m * Rddelrm;
    const std::complex<double> para9 = 36028797018963968 * R * m * Rddelrm * para5;
    const std::complex<double> ret = std::pow(((6.0 * Rddel) / Rddelrm - (2.0 * Rddel) / Rddelrm + std::pow(para1, 2) / (4.0 * R2 * m2 * std::pow(Rddelrm, 2)) + para6 / para8 - para7 / para9), 0.5) * 0.5 - std::pow(((2.0 * Rddel) / Rddelrm - ((2.0 * (2.0 * R * del + R * rm + 2.0 * d * del + d * rm - 2.0 * del * rm - 2.0 * del2 - 4.0 * R2 * m2 - 2.0 * del2 * m2 + d * m2 * rm - 2.0 * del * m2 * rm - 4.0 * R * d * m2 + 6.0 * R * del * m2 + 3.0 * R * m2 * rm + 2.0 * d * del * m2)) / (R * m * Rddelrm) + std::pow(para1, 3.0) / (4.0 * R3 * m3 * std::pow(Rddelrm, 3)) + (6.0 * (Rddel)*para1) / (R * m * std::pow(Rddelrm, 2))) / std::pow(((6.0 * Rddel) / Rddelrm - (2.0 * Rddel) / Rddelrm + std::pow(para1, 2.0) / (4.0 * R2 * m2 * std::pow(Rddelrm, 2.0)) + para6 / para8 - para7 / para9), 0.5) + (6.0 * Rddel) / Rddelrm + std::pow(para1, 2.0) / (2.0 * R2 * m2 * std::pow(Rddelrm, 2)) - para6 / para8 + para7 / para9), 0.5) * 0.5 - para1 / (4 * R * m * Rddelrm);
    return ret.real();
  }

  Vector2<double> sphere2Plane(const TransformStaticParameters &static_params, const SphericalCoordinate &params)
  {
    double d = static_params.h + sqrt(square(static_params.R) - square(static_params.r_o));
    double del = static_params.R - sqrt(square(static_params.R) - square(static_params.r_m));
    double x = mTot(tan(params.theta / 2.0), d, static_params.r_m, static_params.R, del);
    double phi = 2.0 * atan(x);
    phi = atan(sin(phi) / (((static_params.R - del) + d) / static_params.r_m - cos(phi)));
    const double l1 = static_params.L * tan(static_params.epsilon - phi);
    const double l2 = static_params.L * tan(static_params.epsilon + phi);

    del = (-l1 + l2) / 2.0;
    phi = del * sin(static_params.epsilon) / cos(phi);
    phi = sqrt(square(del) - square(phi));
    x = cos(params.psi - M_PI / 2.0);
    d = sin(params.psi - M_PI / 2.0);
    phi = sqrt(1.0 / (square(x) / square(del) + square(d) / square(phi)));
    
    return Vector2<double>(
      (-l1 - l2) / 2.0 + phi * cos(params.psi - M_PI / 2.0),
      phi * sin(params.psi - M_PI / 2.0)
    );
  }

  Cache<TransformStaticParameters, Vector2<double>> P1_CACHE;
  Cache<TransformStaticParameters, Vector2<double>> P2_CACHE;
}

bool TransformStaticParameters::operator ==(const TransformStaticParameters &other) const
{
  return (
    R == other.R
    && r_m == other.r_m
    && r_o == other.r_o
    && h == other.h
    && L == other.L
    && epsilon == other.epsilon
    && delta == other.delta
    && screen_size == other.screen_size
  );
}

bool TransformStaticParameters::operator !=(const TransformStaticParameters &other) const
{
  return (
    R != other.R
    || r_m != other.r_m
    || r_o != other.r_o
    || h != other.h
    || L != other.L
    || epsilon != other.epsilon
    || delta != other.delta
    || screen_size != other.screen_size
  );
}

const TransformStaticParameters TransformStaticParameters::DEFAULT = {
  .R = 4.0,
  .r_m = 1.5,
  .r_o = 2.0,
  .h = 1.0,
  .L = 8.4476252817457738,
  .epsilon = 0.059068559067049511,
  .delta = {
    .x = -2.9,
    .y = -1.6
  },
  .screen_size = {
    .x = 1920,
    .y = 1080
  }
};

SphericalCoordinate::SphericalCoordinate()
  : theta(0.0)
  , psi(0.0)
{
}

SphericalCoordinate::SphericalCoordinate(const double theta, const double psi)
  : theta(theta)
  , psi(psi)
{
}

const SphericalCoordinate SphericalCoordinate::CENTER(0.45 * M_PI, 1.55);

Vector2<double> quori_face::transform(const TransformStaticParameters &static_params, const SphericalCoordinate &coord)
{
  const double x = 40.0 / 3.0;
  const double y = 7.5;
  const double s = 10.8 / 7.5;
  double px1 = 898.62;
  double py1 = 182.37;

  const auto &p1 = P1_CACHE.getOrCompute(static_params, [](const TransformStaticParameters &static_params) {
    const static SphericalCoordinate p1_params(M_PI / 6.0, M_PI);
    return sphere2Plane(static_params, p1_params);
  });

  const auto &p2 = P2_CACHE.getOrCompute(static_params, [](const TransformStaticParameters &static_params) {
    const static SphericalCoordinate p2_params(M_PI / 2.0, M_PI * 2.0 / 3.0);
    return sphere2Plane(static_params, p2_params);
  });

  const auto p = sphere2Plane(static_params, coord);
  Vector2<double> a(-159.78000000000009 / (p1.x - p2.x), -248.13 / (p1.y - p2.y));

  return Vector2<double>(
    s * (a.x * p.x + (px1 - a.x * p1.x)) + static_params.delta.x * static_params.screen_size.x / x,
    s * (a.y * p.y + (py1 - a.y * p1.y)) + static_params.delta.y * static_params.screen_size.y / y 
  );
}

float *quori_face::generateLookupTable(const TransformStaticParameters &static_params, const SphericalCoordinate &min, const SphericalCoordinate &max, const Vector2<std::uint32_t> &size)
{
  float *const lookup_table = new float[static_params.screen_size.x * static_params.screen_size.y * 3];
  const double theta_range = max.theta - min.theta;
  const double psi_range = max.psi - min.psi;

  const auto generate_range = [&](const std::size_t begin_y, const std::size_t end_y) {
    for (std::size_t y = begin_y; y < end_y; ++y)
    {
      for (std::size_t x = 0; x < size.x; ++x)
      {
        const auto t = transform(static_params, SphericalCoordinate(
          min.theta + static_cast<double>(x) / size.x * theta_range,
          min.psi + static_cast<double>(y) / size.y * psi_range
        ));

        const auto tx = clamp<std::size_t>(0, t.x, static_params.screen_size.x - 1);
        const auto ty = clamp<std::size_t>(0, t.y, static_params.screen_size.y - 1);


        const std::size_t index = (ty * static_params.screen_size.x + tx) * 3;
        lookup_table[index + 0] = (float)x / size.x;
        lookup_table[index + 1] = (float)y / size.y;
        lookup_table[index + 2] = 0;
      }
    }
  };

  const std::uint32_t num_threads = std::thread::hardware_concurrency();

  std::vector<std::thread> threads;

  for (std::uint32_t i = 0; i < num_threads; ++i)
  {
    const std::size_t begin_y = (size.y / num_threads) * i;
    const std::size_t end_y = std::min((size.y / num_threads) * (i + 1), size.y);
    threads.emplace_back(std::bind(generate_range, begin_y, end_y));
  }

  for (auto &thread : threads) {
    thread.join();
  }

  return lookup_table;
}
