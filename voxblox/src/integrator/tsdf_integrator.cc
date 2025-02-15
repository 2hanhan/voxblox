#include "voxblox/integrator/tsdf_integrator.h"

#include <iostream>
#include <list>

namespace voxblox {

TsdfIntegratorBase::Ptr TsdfIntegratorFactory::create(
    const std::string& integrator_type_name,
    const TsdfIntegratorBase::Config& config, Layer<TsdfVoxel>* layer) {
  CHECK(!integrator_type_name.empty());

  int integrator_type = 1;
  for (const std::string& valid_integrator_type_name :
       kTsdfIntegratorTypeNames) {
    if (integrator_type_name == valid_integrator_type_name) {
      return create(static_cast<TsdfIntegratorType>(integrator_type), config,
                    layer);
    }
    ++integrator_type;
  }
  LOG(FATAL) << "Unknown TSDF integrator type: " << integrator_type_name;
  return TsdfIntegratorBase::Ptr();
}

TsdfIntegratorBase::Ptr TsdfIntegratorFactory::create(
    const TsdfIntegratorType integrator_type,
    const TsdfIntegratorBase::Config& config, Layer<TsdfVoxel>* layer) {
  CHECK_NOTNULL(layer);
  switch (integrator_type) {
    case TsdfIntegratorType::kSimple:
      return TsdfIntegratorBase::Ptr(new SimpleTsdfIntegrator(config, layer));
      break;
    case TsdfIntegratorType::kMerged:
      return TsdfIntegratorBase::Ptr(new MergedTsdfIntegrator(config, layer));
      break;
    case TsdfIntegratorType::kFast:
      return TsdfIntegratorBase::Ptr(new FastTsdfIntegrator(config, layer));
      break;
    default:
      LOG(FATAL) << "Unknown TSDF integrator type: "
                 << static_cast<int>(integrator_type);
      break;
  }
  return TsdfIntegratorBase::Ptr();
}

// Note many functions state if they are thread safe. Unless explicitly stated
// otherwise, this thread safety is based on the assumption that any pointers
// passed to the functions point to objects that are guaranteed to not be
// accessed by other threads.

TsdfIntegratorBase::TsdfIntegratorBase(const Config& config,
                                       Layer<TsdfVoxel>* layer)
    : config_(config) {
  setLayer(layer);

  if (config_.integrator_threads == 0) {
    LOG(WARNING) << "Automatic core count failed, defaulting to 1 threads";
    config_.integrator_threads = 1;
  }
  // clearing rays have no utility if voxel_carving is disabled
  if (config_.allow_clear && !config_.voxel_carving_enabled) {
    config_.allow_clear = false;
  }
}

void TsdfIntegratorBase::setLayer(Layer<TsdfVoxel>* layer) {
  CHECK_NOTNULL(layer);

  layer_ = layer;

  voxel_size_ = layer_->voxel_size();
  block_size_ = layer_->block_size();
  voxels_per_side_ = layer_->voxels_per_side();

  voxel_size_inv_ = 1.0 / voxel_size_;
  block_size_inv_ = 1.0 / block_size_;
  voxels_per_side_inv_ = 1.0 / voxels_per_side_;
}

// Will return a pointer to a voxel located at global_voxel_idx in the tsdf
// layer. Thread safe.
// Takes in the last_block_idx and last_block to prevent unneeded map lookups.
// If the block this voxel would be in has not been allocated, a block in
// temp_block_map_ is created/accessed and a voxel from this map is returned
// instead. Unlike the layer, accessing temp_block_map_ is controlled via a
// mutex allowing it to grow during integration.
// These temporary blocks can be merged into the layer later by calling
// updateLayerWithStoredBlocks()
/**
 * @brief 返回指向位于tsdf层中global_voxel_idx处的指针
 * - 根据全局索引获得block
 * - 当前点云的voxel对应临时block
 * @param global_voxel_idx 全局索引
 * @param last_block 上一个体素块
 * @param last_block_idx 上一个体素块索引
 * @return TsdfVoxel*
 */
TsdfVoxel* TsdfIntegratorBase::allocateStorageAndGetVoxelPtr(
    const GlobalIndex& global_voxel_idx, Block<TsdfVoxel>::Ptr* last_block,
    BlockIndex* last_block_idx) {
  DCHECK(last_block != nullptr);
  DCHECK(last_block_idx != nullptr);

  const BlockIndex block_idx =
      getBlockIndexFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_inv_);

  if ((block_idx != *last_block_idx) || (*last_block == nullptr)) {
    *last_block = layer_->getBlockPtrByIndex(block_idx);
    *last_block_idx = block_idx;
  }

  // If no block at this location currently exists, we allocate a temporary
  // voxel that will be merged into the map later
  //如果此位置当前不存在块，分配一个临时体素，稍后将其合并到地图中
  if (*last_block == nullptr) {
    // To allow temp_block_map_ to grow we can only let one thread in at once
    std::lock_guard<std::mutex> lock(temp_block_mutex_);

    typename Layer<TsdfVoxel>::BlockHashMap::iterator it =
        temp_block_map_.find(block_idx);
    if (it != temp_block_map_.end()) {
      *last_block = it->second;  //取出对应的block
    } else {
      // emplace(key,value)插入元素，没有先创建后拷贝的操作效率更高
      auto insert_status = temp_block_map_.emplace(
          block_idx, std::make_shared<Block<TsdfVoxel>>(
                         voxels_per_side_, voxel_size_,
                         getOriginPointFromGridIndex(block_idx, block_size_)));

      DCHECK(insert_status.second) << "Block already exists when allocating at "
                                   << block_idx.transpose();

      *last_block = insert_status.first->second;  //取出新创建的block
    }
  }

  (*last_block)->updated().set();

  const VoxelIndex local_voxel_idx =
      getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

  return &((*last_block)->getVoxelByVoxelIndex(local_voxel_idx));
}

/**
 * @brief 更新临时的temp_block_map_到tsdf地图对应的layer_
 * NOT thread safe
 */
void TsdfIntegratorBase::updateLayerWithStoredBlocks() {
  BlockIndex last_block_idx;
  Block<TsdfVoxel>::Ptr block = nullptr;

  for (const std::pair<const BlockIndex, Block<TsdfVoxel>::Ptr>&
           temp_block_pair : temp_block_map_) {
    layer_->insertBlock(temp_block_pair);  //将临时的block更新到对应的tsdf地图
  }

  temp_block_map_.clear();  //情况当前点云生成的临时block地图
}

/**
 * @brief Updates tsdf_voxel. Thread safe.
 * 计算tsdf更新临时的blocks中的voxel
 * @param origin 相机原点
 * @param point_G 世界坐标系下的点
 * @param global_voxel_idx 全局的voxel索引
 * @param color 颜色
 * @param weight 权重
 * @param tsdf_voxel
 */
void TsdfIntegratorBase::updateTsdfVoxel(const Point& origin,
                                         const Point& point_G,
                                         const GlobalIndex& global_voxel_idx,
                                         const Color& color, const float weight,
                                         TsdfVoxel* tsdf_voxel) {
  DCHECK(tsdf_voxel != nullptr);

  //计算voxel中心的x,y,z坐标
  const Point voxel_center =
      getCenterPointFromGridIndex(global_voxel_idx, voxel_size_);

  //计算获得sdf。
  const float sdf = computeDistance(origin, point_G, voxel_center);

  float updated_weight = weight;
  // Compute updated weight in case we use weight dropoff. It's easier here
  // that in getVoxelWeight as here we have the actual SDF for the voxel
  // already computed
  //更新voxel的权重.
  const FloatingPoint dropoff_epsilon = voxel_size_;
  if (config_.use_weight_dropoff && sdf < -dropoff_epsilon) {
    updated_weight = weight * (config_.default_truncation_distance + sdf) /
                     (config_.default_truncation_distance - dropoff_epsilon);
    updated_weight = std::max(updated_weight, 0.0f);
  }

  // Compute the updated weight in case we compensate for sparsity. By
  // multiplicating the weight of occupied areas (|sdf| < truncation distance)
  // by a factor, we prevent to easily fade out these areas with the free
  // space parts of other rays which pass through the corresponding voxels.
  // This can be useful for creating a TSDF map from sparse sensor data (e.g.
  // visual features from a SLAM system). By default, this option is disabled.
  //通过将占用区域的权重（| sdf |<截断距离）乘以一个因子，应对稀疏地图点云的情况
  if (config_.use_sparsity_compensation_factor) {
    if (std::abs(sdf) < config_.default_truncation_distance) {
      updated_weight *= config_.sparsity_compensation_factor;
    }
  }

  // Lookup the mutex that is responsible for this voxel and lock it
  std::lock_guard<std::mutex> lock(mutexes_.get(global_voxel_idx));

  const float new_weight = tsdf_voxel->weight + updated_weight;

  // it is possible to have weights very close to zero, due to the limited
  // precision of floating points dividing by this small value can cause nans
  if (new_weight < kFloatEpsilon) {
    return;
  }

  const float new_sdf =
      (sdf * updated_weight + tsdf_voxel->distance * tsdf_voxel->weight) /
      new_weight;

  // color blending is expensive only do it close to the surface
  //更新rgb颜色(如果有的话)，并对sdf进行截断得到tsdf
  if (std::abs(sdf) < config_.default_truncation_distance) {
    tsdf_voxel->color = Color::blendTwoColors(
        tsdf_voxel->color, tsdf_voxel->weight, color, updated_weight);
  }
  //截断距离
  tsdf_voxel->distance = (new_sdf > 0.0)
                             ? std::min(config_.default_truncation_distance,
                                        new_sdf)  //大于0的情况，取最小的
                             : std::max(-config_.default_truncation_distance,
                                        new_sdf);  //小于0的情况，取最大的
  tsdf_voxel->weight = std::min(config_.max_weight, new_weight);
}

// Thread safe.
// Figure out whether the voxel is behind or in front of the surface.
// To do this, project the voxel_center onto the ray from origin to point G.
// Then check if the the magnitude of the vector is smaller or greater than
// the original distance...

/**
 * @brief
 * 计算sdf值判断，voxel是在平面的前面还是后面。
 * 具体方法是将voxel中心投影到点构成的射线上，
 * 计算二者的差值
 * @param origin
 * @param point_G
 * @param voxel_center
 * @return float sdf值
 */
float TsdfIntegratorBase::computeDistance(const Point& origin,
                                          const Point& point_G,
                                          const Point& voxel_center) const {
  const Point v_voxel_origin = voxel_center - origin;
  const Point v_point_origin = point_G - origin;

  const FloatingPoint dist_G = v_point_origin.norm();
  // projection of a (v_voxel_origin) onto b (v_point_origin)
  // voxel中心的射线投影到直接的点射线
  const FloatingPoint dist_G_V = v_voxel_origin.dot(v_point_origin) / dist_G;

  //点的减去voxel中心的，大于0voxel在表面上，否则
  const float sdf = static_cast<float>(dist_G - dist_G_V);
  return sdf;
}

// Thread safe.
/**
 * @brief 计算单个点权重
 *
 * @param point_C
 * @return float
 */
float TsdfIntegratorBase::getVoxelWeight(const Point& point_C) const {
  if (config_.use_const_weight) {
    return 1.0f;
  }
  const FloatingPoint dist_z = std::abs(point_C.z());
  if (dist_z > kEpsilon) {
    return 1.0f / (dist_z * dist_z);
  }
  return 0.0f;
}

/**
 * @brief 普通方法对每一个图像点都投影射线
 *
 * @param T_G_C
 * @param points_C
 * @param colors
 * @param freespace_points
 */
void SimpleTsdfIntegrator::integratePointCloud(const Transformation& T_G_C,
                                               const Pointcloud& points_C,
                                               const Colors& colors,
                                               const bool freespace_points) {
  timing::Timer integrate_timer("integrate/simple");
  CHECK_EQ(points_C.size(), colors.size());

  std::unique_ptr<ThreadSafeIndex> index_getter(
      ThreadSafeIndexFactory::get(config_.integration_order_mode, points_C));

  std::list<std::thread> integration_threads;
  for (size_t i = 0; i < config_.integrator_threads; ++i) {
    integration_threads.emplace_back(&SimpleTsdfIntegrator::integrateFunction,
                                     this, T_G_C, points_C, colors,
                                     freespace_points, index_getter.get());
  }

  for (std::thread& thread : integration_threads) {
    thread.join();
  }
  integrate_timer.Stop();

  timing::Timer insertion_timer("inserting_missed_blocks");
  updateLayerWithStoredBlocks();
  insertion_timer.Stop();
}

void SimpleTsdfIntegrator::integrateFunction(const Transformation& T_G_C,
                                             const Pointcloud& points_C,
                                             const Colors& colors,
                                             const bool freespace_points,
                                             ThreadSafeIndex* index_getter) {
  DCHECK(index_getter != nullptr);

  size_t point_idx;
  while (index_getter->getNextIndex(&point_idx)) {
    const Point& point_C = points_C[point_idx];
    const Color& color = colors[point_idx];
    bool is_clearing;
    if (!isPointValid(point_C, freespace_points, &is_clearing)) {
      continue;
    }

    const Point origin = T_G_C.getPosition();
    const Point point_G = T_G_C * point_C;

    RayCaster ray_caster(origin, point_G, is_clearing,
                         config_.voxel_carving_enabled,
                         config_.max_ray_length_m, voxel_size_inv_,
                         config_.default_truncation_distance);

    Block<TsdfVoxel>::Ptr block = nullptr;
    BlockIndex block_idx;
    GlobalIndex global_voxel_idx;
    while (ray_caster.nextRayIndex(&global_voxel_idx)) {
      TsdfVoxel* voxel =
          allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);

      const float weight = getVoxelWeight(point_C);

      updateTsdfVoxel(origin, point_G, global_voxel_idx, color, weight, voxel);
    }
  }
}

/**
 * @brief
 * - Merged方法先计算哪些3d点投影到了相同的voxel里，
 * 然后取这些点的坐标平均数，视为一个点，只从那个平均点投影射线更新。
 *
 * @param T_G_C
 * @param points_C
 * @param colors
 * @param freespace_points
 */
void MergedTsdfIntegrator::integratePointCloud(const Transformation& T_G_C,
                                               const Pointcloud& points_C,
                                               const Colors& colors,
                                               const bool freespace_points) {
  timing::Timer integrate_timer("integrate/merged");
  CHECK_EQ(points_C.size(), colors.size());

  // Pre-compute a list of unique voxels to end on.
  // Create a hashmap: VOXEL INDEX -> index in original cloud.
  LongIndexHashMapType<AlignedVector<size_t>>::type voxel_map;
  // This is a hash map (same as above) to all the indices that need to be
  // cleared.
  LongIndexHashMapType<AlignedVector<size_t>>::type clear_map;

  std::unique_ptr<ThreadSafeIndex> index_getter(
      ThreadSafeIndexFactory::get(config_.integration_order_mode, points_C));

  bundleRays(T_G_C, points_C, freespace_points, index_getter.get(), &voxel_map,
             &clear_map);

  integrateRays(T_G_C, points_C, colors, config_.enable_anti_grazing, false,
                voxel_map, clear_map);

  timing::Timer clear_timer("integrate/clear");

  integrateRays(T_G_C, points_C, colors, config_.enable_anti_grazing, true,
                voxel_map, clear_map);

  clear_timer.Stop();

  integrate_timer.Stop();
}

/**
 * @brief 把一些属于同一个voxel的point进行捆绑近似
 *
 * @param T_G_C
 * @param points_C
 * @param freespace_points
 * @param index_getter
 * @param voxel_map
 * @param clear_map
 */
void MergedTsdfIntegrator::bundleRays(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const bool freespace_points, ThreadSafeIndex* index_getter,
    LongIndexHashMapType<AlignedVector<size_t>>::type* voxel_map,
    LongIndexHashMapType<AlignedVector<size_t>>::type* clear_map) {
  DCHECK(voxel_map != nullptr);
  DCHECK(clear_map != nullptr);

  //遍历点云获得世界坐标系下的点
  size_t point_idx;
  while (index_getter->getNextIndex(&point_idx)) {
    const Point& point_C = points_C[point_idx];
    bool is_clearing;
    //距离相机太近或太远之类，不Valid
    if (!isPointValid(point_C, freespace_points, &is_clearing)) {
      continue;  //清除不valid的点
    }

    const Point point_G = T_G_C * point_C;  // 转换到世界坐标系下

    GlobalIndex voxel_index = getGridIndexFromPoint<GlobalIndex>(
        point_G, voxel_size_inv_);  //计算世界坐标系下的点在哪个voxel里

    if (is_clearing) {
      (*clear_map)[voxel_index].push_back(point_idx);
    } else {
      (*voxel_map)[voxel_index].push_back(
          point_idx);  // 投影到一个voxel_index下的不同poin_idx都会被push到voxel_map里。
    }
  }

  VLOG(3) << "Went from " << points_C.size() << " points to "
          << voxel_map->size() << " raycasts  and " << clear_map->size()
          << " clear rays.";
}

/**
 * @brief 遍历一个voxel中所有的点，求取平均参数，进行tsdf的voxel生成更新
 * - 计算射线投影遍历体素，进行tsdf更新
 * @param T_G_C
 * @param points_C
 * @param colors
 * @param enable_anti_grazing
 * @param clearing_ray 当前voxel中是否只取第一个点
 * @param kv *it voxel_map中的一个voxel
 * @param voxel_map
 */
void MergedTsdfIntegrator::integrateVoxel(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
    const std::pair<GlobalIndex, AlignedVector<size_t>>& kv,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map) {
  if (kv.second.empty()) {
    return;
  }

  const Point& origin = T_G_C.getPosition();
  Color merged_color;
  Point merged_point_C = Point::Zero();
  FloatingPoint merged_weight = 0.0;

  //求得属于一个voxel内的点的平均点和颜色
  for (const size_t pt_idx : kv.second) {
    const Point& point_C = points_C[pt_idx];
    const Color& color = colors[pt_idx];

    const float point_weight = getVoxelWeight(point_C);
    if (point_weight < kEpsilon) {
      continue;
    }
    merged_point_C = (merged_point_C * merged_weight + point_C * point_weight) /
                     (merged_weight + point_weight);
    merged_color =
        Color::blendTwoColors(merged_color, merged_weight, color, point_weight);
    merged_weight += point_weight;

    // only take first point when clearing
    // clearing_ray为真只处理第一个点
    if (clearing_ray) {
      break;
    }
  }

  const Point merged_point_G = T_G_C * merged_point_C;  //加权后voxel内点的位姿

  //利用平均点投影射线
  //构造函数，计算射线起始点终点、计算射线步长
  RayCaster ray_caster(origin, merged_point_G, clearing_ray,
                       config_.voxel_carving_enabled, config_.max_ray_length_m,
                       voxel_size_inv_, config_.default_truncation_distance);

  GlobalIndex global_voxel_idx;

  //获取体素的全局索引
  while (ray_caster.nextRayIndex(&global_voxel_idx)) {
    if (enable_anti_grazing) {
      // Check if this one is already the the block hash map for this
      // insertion. Skip this to avoid grazing.
      //
      if ((clearing_ray || global_voxel_idx != kv.first)  // first point会跳过
          && voxel_map.find(global_voxel_idx) !=
                 voxel_map.end())  //如果已经存在hashmap映射则跳过
      {
        continue;
      }
    }

    Block<TsdfVoxel>::Ptr block = nullptr;
    BlockIndex block_idx;
    //返回由输入点云生成的在这个block里的对应index的voxel
    TsdfVoxel* voxel =
        allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);

    updateTsdfVoxel(origin, merged_point_G, global_voxel_idx, merged_color,
                    merged_weight, voxel);
  }
}

/**
 * @brief 遍历所有的voxels
 * - merged方式的遍历所有的voxels
 * @param T_G_C
 * @param points_C
 * @param colors
 * @param enable_anti_grazing
 * @param clearing_ray 当前voxel中是否只取第一个点
 * @param voxel_map
 * @param clear_map
 * @param thread_idx
 */
void MergedTsdfIntegrator::integrateVoxels(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map,
    size_t thread_idx) {
  LongIndexHashMapType<AlignedVector<size_t>>::type::const_iterator it;
  size_t map_size;
  if (clearing_ray) {
    //判断clearing ray标志为
    it = clear_map.begin();
    map_size = clear_map.size();
  } else {
    it = voxel_map.begin();
    map_size = voxel_map.size();
  }

  for (size_t i = 0; i < map_size; ++i) {
    //个数%线程数，每个线程间隔线程数个个数取点
    if (((i + thread_idx + 1) % config_.integrator_threads) == 0) {
      integrateVoxel(T_G_C, points_C, colors, enable_anti_grazing, clearing_ray,
                     *it, voxel_map);
    }
    ++it;
  }
}

/**
 * @brief 多线程的射线投影
 * - merged方式的多线程射线投影
 * @param T_G_C 当前点云坐标变换
 * @param points_C 点云
 * @param colors 颜色信息
 * @param enable_anti_grazing
 * @param clearing_ray
 * @param voxel_map
 * @param clear_map 当前voxel中是否只取第一个点
 */
void MergedTsdfIntegrator::integrateRays(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Colors& colors, bool enable_anti_grazing, bool clearing_ray,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& voxel_map,
    const LongIndexHashMapType<AlignedVector<size_t>>::type& clear_map) {
  // if only 1 thread just do function call, otherwise spawn threads
  if (config_.integrator_threads == 1) {
    constexpr size_t thread_idx = 0;
    integrateVoxels(T_G_C, points_C, colors, enable_anti_grazing, clearing_ray,
                    voxel_map, clear_map, thread_idx);
  } else {
    std::list<std::thread> integration_threads;
    for (size_t i = 0; i < config_.integrator_threads; ++i) {
      integration_threads.emplace_back(
          &MergedTsdfIntegrator::integrateVoxels, this, T_G_C, points_C, colors,
          enable_anti_grazing, clearing_ray, voxel_map, clear_map, i);
    }

    for (std::thread& thread : integration_threads) {
      thread.join();
    }
  }

  timing::Timer insertion_timer("inserting_missed_blocks");
  updateLayerWithStoredBlocks();

  insertion_timer.Stop();
}

void FastTsdfIntegrator::integrateFunction(const Transformation& T_G_C,
                                           const Pointcloud& points_C,
                                           const Colors& colors,
                                           const bool freespace_points,
                                           ThreadSafeIndex* index_getter) {
  DCHECK(index_getter != nullptr);

  size_t point_idx;
  while (index_getter->getNextIndex(&point_idx) &&
         (std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - integration_start_time_)
              .count() < config_.max_integration_time_s * 1000000)) {
    const Point& point_C = points_C[point_idx];
    const Color& color = colors[point_idx];
    bool is_clearing;
    if (!isPointValid(point_C, freespace_points, &is_clearing)) {
      continue;
    }

    const Point origin = T_G_C.getPosition();
    const Point point_G = T_G_C * point_C;
    // Checks to see if another ray in this scan has already started 'close'
    // to this location. If it has then we skip ray casting this point. We
    // measure if a start location is 'close' to another points by inserting
    // the point into a set of voxels. This voxel set has a resolution
    // start_voxel_subsampling_factor times higher then the voxel size.
    GlobalIndex global_voxel_idx;
    global_voxel_idx = getGridIndexFromPoint<GlobalIndex>(
        point_G, config_.start_voxel_subsampling_factor * voxel_size_inv_);
    if (!start_voxel_approx_set_.replaceHash(global_voxel_idx)) {
      continue;
    }

    constexpr bool cast_from_origin = false;
    RayCaster ray_caster(origin, point_G, is_clearing,
                         config_.voxel_carving_enabled,
                         config_.max_ray_length_m, voxel_size_inv_,
                         config_.default_truncation_distance, cast_from_origin);

    int64_t consecutive_ray_collisions = 0;

    Block<TsdfVoxel>::Ptr block = nullptr;
    BlockIndex block_idx;
    while (ray_caster.nextRayIndex(&global_voxel_idx)) {
      // Check if the current voxel has been seen by any ray cast this scan.
      // If it has increment the consecutive_ray_collisions counter, otherwise
      // reset it. If the counter reaches a threshold we stop casting as the
      // ray is deemed to be contributing too little new information.
      if (!voxel_observed_approx_set_.replaceHash(global_voxel_idx)) {
        ++consecutive_ray_collisions;
      } else {
        consecutive_ray_collisions = 0;
      }
      if (consecutive_ray_collisions > config_.max_consecutive_ray_collisions) {
        break;
      }

      TsdfVoxel* voxel =
          allocateStorageAndGetVoxelPtr(global_voxel_idx, &block, &block_idx);

      const float weight = getVoxelWeight(point_C);

      updateTsdfVoxel(origin, point_G, global_voxel_idx, color, weight, voxel);
    }
  }
}

/**
 * @brief
 *
 * @param T_G_C
 * @param points_C
 * @param colors
 * @param freespace_points
 */
void FastTsdfIntegrator::integratePointCloud(const Transformation& T_G_C,
                                             const Pointcloud& points_C,
                                             const Colors& colors,
                                             const bool freespace_points) {
  timing::Timer integrate_timer("integrate/fast");
  CHECK_EQ(points_C.size(), colors.size());

  integration_start_time_ = std::chrono::steady_clock::now();

  static int64_t reset_counter = 0;
  if ((++reset_counter) >= config_.clear_checks_every_n_frames) {
    reset_counter = 0;
    start_voxel_approx_set_.resetApproxSet();
    voxel_observed_approx_set_.resetApproxSet();
  }

  std::unique_ptr<ThreadSafeIndex> index_getter(
      ThreadSafeIndexFactory::get(config_.integration_order_mode, points_C));

  std::list<std::thread> integration_threads;
  for (size_t i = 0; i < config_.integrator_threads; ++i) {
    integration_threads.emplace_back(&FastTsdfIntegrator::integrateFunction,
                                     this, T_G_C, points_C, colors,
                                     freespace_points, index_getter.get());
  }

  for (std::thread& thread : integration_threads) {
    thread.join();
  }

  integrate_timer.Stop();

  timing::Timer insertion_timer("inserting_missed_blocks");
  updateLayerWithStoredBlocks();
  insertion_timer.Stop();
}

std::string TsdfIntegratorBase::Config::print() const {
  std::stringstream ss;
  // clang-format off
  ss << "================== TSDF Integrator Config ====================\n";
  ss << " General: \n";
  ss << " - default_truncation_distance:               " << default_truncation_distance << "\n";
  ss << " - max_weight:                                " << max_weight << "\n";
  ss << " - voxel_carving_enabled:                     " << voxel_carving_enabled << "\n";
  ss << " - min_ray_length_m:                          " << min_ray_length_m << "\n";
  ss << " - max_ray_length_m:                          " << max_ray_length_m << "\n";
  ss << " - use_const_weight:                          " << use_const_weight << "\n";
  ss << " - allow_clear:                               " << allow_clear << "\n";
  ss << " - use_weight_dropoff:                        " << use_weight_dropoff << "\n";
  ss << " - use_sparsity_compensation_factor:          " << use_sparsity_compensation_factor << "\n";
  ss << " - sparsity_compensation_factor:              "  << sparsity_compensation_factor << "\n";
  ss << " - integrator_threads:                        " << integrator_threads << "\n";
  ss << " MergedTsdfIntegrator: \n";
  ss << " - enable_anti_grazing:                       " << enable_anti_grazing << "\n";
  ss << " FastTsdfIntegrator: \n";
  ss << " - start_voxel_subsampling_factor:            " << start_voxel_subsampling_factor << "\n";
  ss << " - max_consecutive_ray_collisions:            " << max_consecutive_ray_collisions << "\n";
  ss << " - clear_checks_every_n_frames:               " << clear_checks_every_n_frames << "\n";
  ss << " - max_integration_time_s:                    " << max_integration_time_s << "\n";
  ss << "==============================================================\n";
  // clang-format on
  return ss.str();
}

}  // namespace voxblox
