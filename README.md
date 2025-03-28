# IESKF SLAM :基于滤波器的前端里程计以及Ceres优化的后端
这是一个基于IESKF(迭代卡尔曼滤波器)编写的SLAM框架，在前端采用了FAST-LIO的理论部分设计了前端里程计。后端部分，采用Scan-Context进行回环检测，并使用Ceres进行位姿图优化。

目录页可以参见：

https://zhuanlan.zhihu.com/p/635702243

## 1 点击分支查看不同分支下的代码：

### 1.1 前端部分：

**v1**:https://zhuanlan.zhihu.com/p/636020378 工程框架建立
[v1]: https://github.com/mengkai98/ieskf_slam/tree/v1

**v2**:https://zhuanlan.zhihu.com/p/636020378 初始化部分
[v2]: https://github.com/mengkai98/ieskf_slam/tree/v2

**v3**:https://zhuanlan.zhihu.com/p/636020378 前向传播
[v3]: https://github.com/mengkai98/ieskf_slam/tree/v3

**v4**:https://zhuanlan.zhihu.com/p/636020378 ieskf更新
[v4]: https://github.com/mengkai98/ieskf_slam/tree/v4

**v5**:https://zhuanlan.zhihu.com/p/636020378 地图更新
[v5]: https://github.com/mengkai98/ieskf_slam/tree/v5

**v6**:https://zhuanlan.zhihu.com/p/636020378 后向传播
[v6]: https://github.com/mengkai98/ieskf_slam/tree/v6

**v7**:https://zhuanlan.zhihu.com/p/636020378 M2DGR适配以及轨迹评估
[v7]: https://github.com/mengkai98/ieskf_slam/tree/v7

### 1.2 后端部分：

**pose_graph**: https://zhuanlan.zhihu.com/p/666030162 动手写Ceres回环优化
[pose_graph]: https://github.com/mengkai98/ieskf_slam/tree/pose_graph


## 2  BUG修正：

2025年3月28日，修复了所有分支里的如下问题：

### 2.1 IESKF 状态收敛判断：

https://github.com/mengkai98/ieskf_slam/issues/2

这里在`update_x`的判断里，误把索引`idx`写成了`i`

需要如下修改：

```c++
// ieskf_slam/src/ieskf_slam/modules/ieskf/ieskf.cpp
// line 99
 converge = true; 
 for (int idx = 0; idx < 18; idx++) { 
     // if (update_x(i, 0) > 0.001) {  错误的
     if (update_x(idx, 0) > 0.001) { // 正确的
         converge = false; 
         break; 
     } 
 } 
```

### 2.2 地图模块中，删除掉地图范围外点的判断：

https://github.com/mengkai98/ieskf_slam/issues/3

在地图管理的这部分，我们应该采用或逻辑来判断点是否是在地图的外面，而不是与逻辑

```c++
// src/ieskf_slam/modules/map/rect_map_manager.cpp
// line 39
while (left < right) {
    while (left < right &&
           // 错误的
           //abs(local_map_ptr->points[right].x - pos_t.x()) > map_side_length_2 &&
           //abs(local_map_ptr->points[right].y - pos_t.y()) > map_side_length_2 &&
           //abs(local_map_ptr->points[right].z - pos_t.z()) > map_side_length_2)
           // 正确的
           abs(local_map_ptr->points[right].x - pos_t.x()) > map_side_length_2 ||
           abs(local_map_ptr->points[right].y - pos_t.y()) > map_side_length_2 ||
           abs(local_map_ptr->points[right].z - pos_t.z()) > map_side_length_2)
        right--;
    while (left < right &&
           abs(local_map_ptr->points[left].x - pos_t.x()) < map_side_length_2 &&
           abs(local_map_ptr->points[left].y - pos_t.y()) < map_side_length_2 &&
           abs(local_map_ptr->points[left].z - pos_t.z()) < map_side_length_2)
        left++;
    std::swap(local_map_ptr->points[left], local_map_ptr->points[right]);
}
```

## 2.3 外参的传入：

很抱歉，在后面的检查中，我发现yaml里的外参根本没有传入，因此要做一些修改：

```c++
//src/ieskf_slam/modules/frondend/frontend.cpp
// line 55
void FrontEnd::addPointCloud(const Frame &frame) {
    frame_deque.push_back(frame);
    // 添加如下这一行
    // 将点云变换到IMU系下：
    pcl::transformPointCloud(*pointcloud_deque.back().cloud_ptr,
                                 *pointcloud_deque.back().cloud_ptr,
                                 compositeTransform(extrin_r, extrin_t).cast<float>());
}
```

