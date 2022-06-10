#ifndef VOXBLOX_CORE_BLOCK_HASH_H_
#define VOXBLOX_CORE_BLOCK_HASH_H_

#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Core>

#include "voxblox/core/common.h"

namespace voxblox {

/**
 * Performs deco hashing on block indexes. Based on recommendations of
 * "Investigating the impact of Suboptimal Hashing Functions" by L. Buckley et
 * al.
 */
struct AnyIndexHash {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// number was arbitrarily chosen with no good justification
  static constexpr size_t sl = 17191;  //数字是任意选择的
  static constexpr size_t sl2 = sl * sl;

  /**
   * @brief (x,y,z)=>hash表
   *
   * @param index  matrix<int,3,1>
   * @return std::size_t
   */
  std::size_t operator()(const AnyIndex& index) const {
    return static_cast<unsigned int>(index.x() + index.y() * sl +
                                     index.z() * sl2);
  }
};

template <typename ValueType>
struct AnyIndexHashMapType {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::unordered_map<
      AnyIndex,                 // 3D坐标  key_type
      ValueType,                // 模板值  mapper_type
      AnyIndexHash,             //  (x,y,z)=>hash表 hasher
      std::equal_to<AnyIndex>,  //比较参数类型是否一致 key_equal
      Eigen::aligned_allocator<
          std::pair<const AnyIndex, ValueType> > >  //参数坐标，模板值  allocator_type<value_type>
      type;
};

typedef std::unordered_set<AnyIndex, AnyIndexHash, std::equal_to<AnyIndex>,
                           Eigen::aligned_allocator<AnyIndex> >
    IndexSet;

typedef typename AnyIndexHashMapType<IndexVector>::type HierarchicalIndexMap;//hashmap存储key和value

typedef typename AnyIndexHashMapType<IndexSet>::type HierarchicalIndexSet;//hashset存储key(value=key)

typedef typename HierarchicalIndexMap::value_type HierarchicalIndex;//key，value

/// Hash for large index values, see AnyIndexHash.
struct LongIndexHash {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr size_t sl = 17191;
  static constexpr size_t sl2 = sl * sl;

  std::size_t operator()(const LongIndex& index) const {
    return static_cast<unsigned int>(index.x() + index.y() * sl +
                                     index.z() * sl2);
  }
};

template <typename ValueType>
struct LongIndexHashMapType {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::unordered_map<
      LongIndex, ValueType, LongIndexHash, std::equal_to<LongIndex>,
      Eigen::aligned_allocator<std::pair<const LongIndex, ValueType> > >
      type;
};

typedef std::unordered_set<LongIndex, LongIndexHash, std::equal_to<LongIndex>,
                           Eigen::aligned_allocator<LongIndex> >
    LongIndexSet;

}  // namespace voxblox

#endif  // VOXBLOX_CORE_BLOCK_HASH_H_
