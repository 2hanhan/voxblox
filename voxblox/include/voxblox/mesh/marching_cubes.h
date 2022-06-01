// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef VOXBLOX_MESH_MARCHING_CUBES_H_
#define VOXBLOX_MESH_MARCHING_CUBES_H_

#include "voxblox/mesh/mesh.h"

namespace voxblox {

/**
 * Performs the marching cubes algorithm to generate a mesh layer from a TSDF.
 * Implementation taken from Open Chisel
 * https://github.com/personalrobotics/OpenChisel
 */
class MarchingCubes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const int kTriangleTable[256][16];
  static const int kEdgeIndexPairs[12][2];

  MarchingCubes() {}
  virtual ~MarchingCubes() {}

  static void meshCube(
      const Eigen::Matrix<FloatingPoint, 3, 8>& vertex_coordinates,
      const Eigen::Matrix<FloatingPoint, 8, 1>& vertex_sdf,
      TriangleVector* triangles) {
    DCHECK(triangles != NULL);

    const int index = calculateVertexConfiguration(vertex_sdf);

    Eigen::Matrix<FloatingPoint, 3, 12> edge_coords;
    interpolateEdgeVertices(vertex_coordinates, vertex_sdf, &edge_coords);

    const int* table_row = kTriangleTable[index];

    int edge_index = 0;
    int table_col = 0;
    while ((edge_index = table_row[table_col]) != -1) {
      Triangle triangle;
      triangle.col(0) = edge_coords.col(edge_index);
      edge_index = table_row[table_col + 1];
      triangle.col(1) = edge_coords.col(edge_index);
      edge_index = table_row[table_col + 2];
      triangle.col(2) = edge_coords.col(edge_index);
      triangles->push_back(triangle);
      table_col += 3;
    }
  }

  static void meshCube(const Eigen::Matrix<FloatingPoint, 3, 8>& vertex_coords,
                       const Eigen::Matrix<FloatingPoint, 8, 1>& vertex_sdf,
                       VertexIndex* next_index, Mesh* mesh) {
    DCHECK(next_index != NULL);
    DCHECK(mesh != NULL);
    //根据8个顶点的sdf,获得一个8位的int常量index，该量上的每一位代表tsdf的正负，如果为正则那一位为1，否则为0
    const int index = calculateVertexConfiguration(vertex_sdf);

    // No edges in this cube.
    //全是0，cube在表面内部
    if (index == 0) {
      return;
    }

    // cube的12条边
    Eigen::Matrix<FloatingPoint, 3, 12> edge_vertex_coordinates;
    // cube 边上的0点计算与线性差值
    interpolateEdgeVertices(vertex_coords, vertex_sdf,
                            &edge_vertex_coordinates);

    const int* table_row = kTriangleTable[index];

    int table_col = 0;
    while (table_row[table_col] != -1) {
      //保存mesh的三个顶点
      mesh->vertices.emplace_back(
          edge_vertex_coordinates.col(table_row[table_col + 2]));
      mesh->vertices.emplace_back(
          edge_vertex_coordinates.col(table_row[table_col + 1]));
      mesh->vertices.emplace_back(
          edge_vertex_coordinates.col(table_row[table_col]));

      // mesh的顶点的index
      mesh->indices.push_back(*next_index);
      mesh->indices.push_back((*next_index) + 1);
      mesh->indices.push_back((*next_index) + 2);
      //计算mesh的法向量
      const Point& p0 = mesh->vertices[*next_index];
      const Point& p1 = mesh->vertices[*next_index + 1];
      const Point& p2 = mesh->vertices[*next_index + 2];
      Point px = (p1 - p0);
      Point py = (p2 - p0);
      Point n = px.cross(py).normalized();
      mesh->normals.push_back(n);
      mesh->normals.push_back(n);
      mesh->normals.push_back(n);
      *next_index += 3;  //指向下一个mesh3个顶点的指针
      table_col += 3;    //当前的cube的过0点的边的指针
    }
  }

  /**
   * @brief 获取每个顶点的stf值的正负，确定平面的位置
   *
   * @param vertex_sdf
   * @return int
   */
  static int calculateVertexConfiguration(
      const Eigen::Matrix<FloatingPoint, 8, 1>& vertex_sdf) {
    // |按照位或 运算符，移位操作
    return (vertex_sdf(0) < 0 ? (1 << 0) : 0) |
           (vertex_sdf(1) < 0 ? (1 << 1) : 0) |
           (vertex_sdf(2) < 0 ? (1 << 2) : 0) |
           (vertex_sdf(3) < 0 ? (1 << 3) : 0) |
           (vertex_sdf(4) < 0 ? (1 << 4) : 0) |
           (vertex_sdf(5) < 0 ? (1 << 5) : 0) |
           (vertex_sdf(6) < 0 ? (1 << 6) : 0) |
           (vertex_sdf(7) < 0 ? (1 << 7) : 0);
  }

  /**
   * @brief 计算cube需要差值的边，计算差值的0点的位置
   *
   * @param vertex_coords
   * @param vertex_sdf
   * @param edge_coords
   */
  static void interpolateEdgeVertices(
      const Eigen::Matrix<FloatingPoint, 3, 8>& vertex_coords,
      const Eigen::Matrix<FloatingPoint, 8, 1>& vertex_sdf,
      Eigen::Matrix<FloatingPoint, 3, 12>* edge_coords) {
    DCHECK(edge_coords != NULL);
    for (std::size_t i = 0; i < 12; ++i) {
      const int* pairs = kEdgeIndexPairs[i];
      const int edge0 = pairs[0];
      const int edge1 = pairs[1];
      // Only interpolate along edges where there is a zero crossing.
      //对有sdf值有越过0变化的边进行差值
      if ((vertex_sdf(edge0) < 0 && vertex_sdf(edge1) >= 0) ||
          (vertex_sdf(edge0) >= 0 && vertex_sdf(edge1) < 0))
        edge_coords->col(i) = interpolateVertex(
            vertex_coords.col(edge0), vertex_coords.col(edge1),
            vertex_sdf(edge0), vertex_sdf(edge1));
    }
  }

  /**
   * @brief Performs linear interpolation on two cube corners to find the
   * approximate zero crossing (surface) value. 进行线性差值获取过0点的边的位置
   * @param vertex1
   * @param vertex2
   * @param sdf1
   * @param sdf2
   * @return Point
   */
  static inline Point interpolateVertex(const Point& vertex1,
                                        const Point& vertex2, float sdf1,
                                        float sdf2) {
    static constexpr FloatingPoint kMinSdfDifference = 1e-6;
    const FloatingPoint sdf_diff = sdf1 - sdf2;
    // Only compute the actual interpolation value if the sdf_difference is not
    // too small, this is to counteract issues with floating point precision.
    if (std::abs(sdf_diff) >= kMinSdfDifference) {
      const FloatingPoint t = sdf1 / sdf_diff;
      return Point(vertex1 + t * (vertex2 - vertex1));
    } else {
      return Point(0.5 * (vertex1 + vertex2));
    }
  }
};

}  // namespace voxblox

#endif  // VOXBLOX_MESH_MARCHING_CUBES_H_
