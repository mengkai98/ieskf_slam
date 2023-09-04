# IESKF SLAM :基于滤波器的前端里程计以及Ceres优化的后端
这是一个基于IESKF(迭代卡尔曼滤波器)编写的SLAM框架，在前端采用了FAST-LIO的理论部分设计了前端里程计。后端部分，采用Scan-Context进行回环检测，并使用Ceres进行位姿图优化。

## 一、 一些BUG：

1. 非常大的一个BUG，里程计里对点云的操作都是在IMU系下的点云进行操作，但实际上，外参并没有被使用，幸好AVIA和M2DGR中lidar和imu的外参旋转上是单位阵，这并没有影响到后序的操作，但是对于外参不是单位阵的数据集就会存在问题，最新版的代码已经修正了这个问题：

```c++
   // 修改与src/ieskf_slam/modules/frondend/frontend.cpp
void FrontEnd::addPointCloud(const Frame &frame) {
    frame_deque.push_back(frame);
    // 将点云变换到IMU系下：
    pcl::transformPointCloud(*frame_deque.back().cloud_ptr,
                           *frame_deque.back().cloud_ptr,
                            Frame::Extrin.toTransformf());
}

```

在v4-v7的版本里要做如下修改：

```c++
//头文件包含
#include <pcl/common/transforms.h>
...
...
void FrontEnd::addPointCloud(const Frame &frame) {
    frame_deque.push_back(frame);
    // 将点云变换到IMU系下：
    pcl::transformPointCloud(*pointcloud_deque.back().cloud_ptr,
                                 *pointcloud_deque.back().cloud_ptr,
                                 compositeTransform(extrin_r, extrin_t).cast<float>());
}
```

   

## 二、前端的部分已经更新完毕：

前端的设计包含7个章节，目前已全部更新完毕，详情见：

https://zhuanlan.zhihu.com/p/635702243

## 三、后端部分

更新了Scan-Context的回环检测部分，以及位姿图优化，相关的文章会在一段时间后整理上传。