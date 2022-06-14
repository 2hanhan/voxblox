#include "voxblox/integrator/integrator_utils.h"

namespace voxblox {

/**
 * @brief  计算哪些点会被投影到一个voxel里
 *
 * @param mode
 * @param points_C
 * @return ThreadSafeIndex*
 */
ThreadSafeIndex* ThreadSafeIndexFactory::get(const std::string& mode,
                                             const Pointcloud& points_C) {
  if (mode == "mixed") {
    return new MixedThreadSafeIndex(points_C.size());
  } else if (mode == "sorted") {
    return new SortedThreadSafeIndex(points_C);
  } else {
    LOG(FATAL) << "Unknown integration order mode: '" << mode << "'!";
  }
  return nullptr;
}

ThreadSafeIndex::ThreadSafeIndex(size_t number_of_points)
    : atomic_idx_(0), number_of_points_(number_of_points) {}

MixedThreadSafeIndex::MixedThreadSafeIndex(size_t number_of_points)
    : ThreadSafeIndex(number_of_points),
      number_of_groups_(number_of_points / step_size_) {}

SortedThreadSafeIndex::SortedThreadSafeIndex(const Pointcloud& points_C)
    : ThreadSafeIndex(points_C.size()) {
  indices_and_squared_norms_.reserve(points_C.size());
  size_t idx = 0;
  for (const Point& point_C : points_C) {
    indices_and_squared_norms_.emplace_back(idx, point_C.squaredNorm());
    ++idx;
  }

  std::sort(
      indices_and_squared_norms_.begin(), indices_and_squared_norms_.end(),
      [](const std::pair<size_t, double>& a,
         const std::pair<size_t, double>& b) { return a.second < b.second; });
}

// returns true if index is valid, false otherwise
bool ThreadSafeIndex::getNextIndex(size_t* idx) {
  DCHECK(idx != nullptr);
  size_t sequential_idx = atomic_idx_.fetch_add(1);

  if (sequential_idx >= number_of_points_) {
    return false;
  } else {
    *idx = getNextIndexImpl(sequential_idx);
    return true;
  }
}

void ThreadSafeIndex::reset() { atomic_idx_.store(0); }

size_t MixedThreadSafeIndex::getNextIndexImpl(size_t sequential_idx) {
  if (number_of_groups_ * step_size_ <= sequential_idx) {
    return sequential_idx;
  }

  const size_t group_num = sequential_idx % number_of_groups_;
  const size_t position_in_group = sequential_idx / number_of_groups_;

  return group_num * step_size_ + position_in_group;
}

size_t SortedThreadSafeIndex::getNextIndexImpl(size_t sequential_idx) {
  return indices_and_squared_norms_[sequential_idx].first;
}

/**
 * @brief Construct a new Ray Caster:: Ray Caster object
 * This class assumes PRE-SCALED coordinates, where one unit = one voxel size.
 * The indices are also returned in this scales coordinate system, which should
 * map to voxel indices.
 * 设置射线投影的起点终点、并计算设置步长
 * @param origin 相机位姿
 * @param point_G 输入点云生成的voxel内所有点加权后的位姿
 * @param is_clearing_ray true
 * 射线终点：表面接近相机Td的位置，false射线终点：表面远离td位置。
 * @param voxel_carving_enabled 射线的开始点为原点(voxel_carving_enabled==true)
 * @param max_ray_length_m 最大射线长度
 * @param voxel_size_inv  输入点云生成的voxel的大小的倒数
 * @param truncation_distance 截断距离
 * @param cast_from_origin 从原点开始，还是从终点开始，voxel索引的范围
 */
RayCaster::RayCaster(const Point& origin, const Point& point_G,
                     const bool is_clearing_ray,
                     const bool voxel_carving_enabled,
                     const FloatingPoint max_ray_length_m,
                     const FloatingPoint voxel_size_inv,
                     const FloatingPoint truncation_distance,
                     const bool cast_from_origin) {
  //计算单位射线的长度，就是简单的求点到原点的差值，求得他们的norm(均方值)再每一个维度除以norm归一化。
  const Ray unit_ray = (point_G - origin).normalized();

  Point ray_start, ray_end;
  if (is_clearing_ray) {
    FloatingPoint ray_length = (point_G - origin).norm();
    ray_length = std::min(
        std::max(
            ray_length -
                truncation_distance,  //射线长度减去截断距离，截断距离就是平面的+-Td的
            static_cast<FloatingPoint>(0.0)),  //和0比较把距离相机太近的去除掉
        max_ray_length_m);  //不能超过射线的最大距离

    //终点：平均点 - unit_ray *
    // truncation_distance的位置。表面距离相机原点近一点
    ray_end = origin + unit_ray * ray_length;
    //是否是从相机开始的射线
    ray_start = voxel_carving_enabled ? origin :  //原点
                    ray_end;                      //表面接近相机Td
  } else {
    // 终点：平均点 + unit_ray * truncation_distance。表面距离相机原点远一点
    // 就是说距离相机原点，超过射线原点还没有检测到平面，需要把原来的平面清除了
    ray_end = point_G + unit_ray * truncation_distance;
    ray_start =
        voxel_carving_enabled
            ? origin                                       //原点
            : (point_G - unit_ray * truncation_distance);  //表面接近相机Td
  }

  // voxel的索引范围，位置/voxel大小
  const Point start_scaled = ray_start * voxel_size_inv;
  const Point end_scaled = ray_end * voxel_size_inv;

  if (cast_from_origin) {
    setupRayCaster(start_scaled, end_scaled);
  } else {
    setupRayCaster(end_scaled, start_scaled);
  }
}

RayCaster::RayCaster(const Point& start_scaled, const Point& end_scaled) {
  setupRayCaster(start_scaled, end_scaled);
}

/**
 * @brief returns false if ray terminates at ray_index, true otherwise
 * 如果搜索完成index就返回false
 * @param ray_index 射线投影获得的voxel的索引
 * @return true
 * @return false
 */
bool RayCaster::nextRayIndex(GlobalIndex* ray_index) {
  //如果射线投影到达最大距离则返回
  if (current_step_++ > ray_length_in_steps_) {
    return false;
  }

  DCHECK(ray_index != nullptr);
  *ray_index = curr_index_;

  //获取t_to_next_boundary_中最小的数的索引
  //每次只取一个轴进行，按照射线方向的步长进行移动
  int t_min_idx;
  t_to_next_boundary_.minCoeff(&t_min_idx);

  curr_index_[t_min_idx] += ray_step_signs_[t_min_idx];  //更新索引

  //更新位置索引增量=+绝对步长
  t_to_next_boundary_[t_min_idx] += t_step_size_[t_min_idx];
  //! 这块就是理解不了
  //! 1. 第一个的 t_to_next_boundary_ 的值就很奇怪
  //! 2. 每次取min的
  //! t_to_next_boundary_的列，t_to_next_boundary_是绝对位置的增量，每次取min列，这射线的方向也不对啊，有的轴增量小把dx加完了也比不上其他的轴一次的大小啊
  return true;
}

/**
 * @brief 设置射线搜索的步长信息
 *
 * @param start_scaled  按照voxel_size_inv计算索引后的射线起点
 * @param end_scaled 按照voxel_size_inv计算索引后的射线终点
 */
void RayCaster::setupRayCaster(const Point& start_scaled,
                               const Point& end_scaled) {
  // 终点or起点坐标异常直接返回
  if (std::isnan(start_scaled.x()) || std::isnan(start_scaled.y()) ||
      std::isnan(start_scaled.z()) || std::isnan(end_scaled.x()) ||
      std::isnan(end_scaled.y()) || std::isnan(end_scaled.z())) {
    ray_length_in_steps_ = 0;
    return;
  }

  //首先获取射线开始位置和结束位置对应的voxel的三个方向的index的坐标以及差值
  curr_index_ = getGridIndexFromPoint<GlobalIndex>(start_scaled);
  const GlobalIndex end_index = getGridIndexFromPoint<GlobalIndex>(end_scaled);
  const GlobalIndex diff_index = end_index - curr_index_;  //索引变化范围

  current_step_ = 0;

  ray_length_in_steps_ = std::abs(diff_index.x()) + std::abs(diff_index.y()) +
                         std::abs(diff_index.z());  //所有的dx+dy+dz的总和

  const Ray ray_scaled = end_scaled - start_scaled;  //射线矢量

  // 每个维度(x,y,z)只可能为3个值, 1,-1或者0.
  //查看射线x，y，z的变化方向，正方向为1，负方向为-1，不变为0
  ray_step_signs_ = AnyIndex(signum(ray_scaled.x()), signum(ray_scaled.y()),
                             signum(ray_scaled.z()));

  //?值保留正的为1，负的设置为0，感觉初始位置index增量就是随便取了一个不为负数就行
  const AnyIndex corrected_step(std::max(0, ray_step_signs_.x()),
                                std::max(0, ray_step_signs_.y()),
                                std::max(0, ray_step_signs_.z()));

  //计算坐标相对于初始位置的偏移量，主要就是取个近似计算呗
  // start_scaled，curr_index_应该是近似相等的
  const Point start_scaled_shifted =
      start_scaled - curr_index_.cast<FloatingPoint>();

  //第一个绝对坐标值增量值
  Ray distance_to_boundaries(corrected_step.cast<FloatingPoint>() -
                             start_scaled_shifted);

  //第一个归一化的坐标增量值
  t_to_next_boundary_ = Ray((std::abs(ray_scaled.x()) < 0.0)
                                ? 2.0
                                : distance_to_boundaries.x() / ray_scaled.x(),
                            (std::abs(ray_scaled.y()) < 0.0)
                                ? 2.0
                                : distance_to_boundaries.y() / ray_scaled.y(),
                            (std::abs(ray_scaled.z()) < 0.0)
                                ? 2.0
                                : distance_to_boundaries.z() / ray_scaled.z());

  // Distance to cross one grid cell along the ray in t.
  // Same as absolute inverse value of delta_coord.
  //[射线矢量的abs<0.0 ]?[2.0]:[射线矢量模的倒数]
  //防止分母过小取倒数为正无穷
  //归一化，坐标增量，dx+dy+dz = 16，结合nextRayIndex函数
  //举个栗子：假设ray_scale =10，5，1
  // x：[0.0][0.1][0.2][0.3][0.4][0.5][0.6][0.7][0.8][0.9][1.0]
  // y：[0.0][.....][0.2][.....][0.4][.....][0.6][.....][0.8][.....][1.0]
  // z：[0.0][.....][.....][.....][.....][.....][.....][.....][.....][.....][1.0]
  //为了使得x，y，z成比例增加，这样才能是直线
  //所以每次增加t_to_next_boundary_中最小的一个
  t_step_size_ = Ray(
      (std::abs(ray_scaled.x()) < 0.0) ? 2.0
                                       : ray_step_signs_.x() / ray_scaled.x(),
      (std::abs(ray_scaled.y()) < 0.0) ? 2.0
                                       : ray_step_signs_.y() / ray_scaled.y(),
      (std::abs(ray_scaled.z()) < 0.0) ? 2.0
                                       : ray_step_signs_.z() / ray_scaled.z());
}

}  // namespace voxblox
