#pragma once
#include <Eigen/Dense>
#include <variant>

namespace qpik {

struct GeometryBox {
  Eigen::Vector3d size;
};

struct GeometryCylinder {
  double radius;
  double height;
};

struct GeometrySphere {
  double radius;
};

struct GeometryMesh {
  std::string filename;
  Eigen::Vector3d scale;
};

using GeometryVariant =
    std::variant<GeometryBox, GeometryCylinder, GeometrySphere, GeometryMesh>;

} // namespace qpik