# 实现odometry变化到transform的消息发布
> 然而调整了半天发现r3live的`sensor_msgs/PointCloud2`话题发布的`sensor_msgs/PointCloud2`的类型的消息本身就是已经变化到`world`坐标系下面了，所有这个就没有做坐标变化的必要了，直接调参就能不错了。
> 不需要进行坐标变化，也不需要ICP的矫正直接生成estf的体素就可以了