#include "geometry.hpp"

const std::array<float, 20> quori_face::GEOMETRY_VERTICES {
   1.0f, -1.0f, 0.0f, 1.0f, 1.0f, // top right
   1.0f,  1.0f, 0.0f, 1.0f, 0.0f, // bottom right
  -1.0f,  1.0f, 0.0f, 0.0f, 0.0f, // bottom left
  -1.0f, -1.0f, 0.0f, 0.0f, 1.0f  // top left 
};

const std::array<std::uint32_t, 6> quori_face::GEOMETRY_INDICES {
  0, 1, 3,
  1, 2, 3
};