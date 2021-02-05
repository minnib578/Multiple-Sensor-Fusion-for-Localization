# Sensor Fusion: Lidar Odometry -- 多传感器融合定位与建图: 激光里程计

深蓝学院, 多传感器融合定位与建图, 第1节Lidar Odometry代码框架.

---

## Overview

本作业旨在实现主流的激光前端里程计算法.

---

## Getting Started

### 及格要求: 跑通提供的工程框架

启动Docker后, 打开浏览器, 进入Web Workspace. 启动Terminator, 将两个Shell的工作目录切换如下:

<img src="doc/terminator.png" alt="Terminator" width="100%">

在**上侧**的Shell中, 输入如下命令, **编译catkin_workspace**

```bash
# build
catkin config --install && catkin build
```

然后**启动解决方案**

```bash
# set up session:
source install/setup.bash
# launch:
roslaunch lidar_localization front_end.launch
```

在**下侧**的Shell中, 输入如下命令, **Play KITTI ROS Bag**. 两个数据集均可用于完成课程, 对代码功能的运行没有任何影响, 区别在于第一个有Camera信息

```bash
# play ROS bag, full KITTI:
rosbag play kitti_2011_10_03_drive_0027_synced.bag
# play ROS bag, lidar-only KITTI:
rosbag play kitti_lidar_only_2011_10_03_drive_0027_synced.bag
```

成功后, 可以看到如下的RViz界面:

<img src="doc/demo.png" alt="Frontend Demo" width="100%">

### 良好要求: 使用evo计算出分段统计误差和整体轨迹误差

此处以Docker Workspace为例. 在Terminator中添加新窗口, 切换至如下目录:

```bash
cd /workspace/assignments/01-lidar-odometry/src/lidar_localization/slam_data/trajectory
```

<img src="doc/trajectory-dump.png" alt="Trajectory Dumps" width="100%">

该目录下会输出:

* 作为Ground Truth的RTK轨迹估计, ground_truth.txt
* Lidar Frontend轨迹估计, laser_odom.txt

请使用上述两个文件, 完成**evo**的评估

### 优秀要求: 自己实现点云匹配方法，而不是直接调用pcl库的匹配方法，并使用evo计算出指标

# 高能预警: 该作业需要足够的C++与ROS开发基础. 如果没有, 欢迎发挥主观能动性, 在有限的时间内积极Catch Up, 或者选修深蓝学院的相关课程, 留待务实基础之后再来学习.

# 学习建议: 此处的参考代码, 只会削弱你的学习收获. 如果有意提高自己, 请在理解框架的基础上, 删除参考实现, 从零开始自己实现. 相信真正热爱学习的你，可以做到!!!

前端里程计的算法, 在[此处](src/lidar_localization/src/front_end/front_end.cpp#L76)添加. **作为示例**, 此处在框架原有的`ICP`, `NDT`两方法的基础上, 增加了新的方法`ICP_SVD`.

```c++
    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else if (registration_method == "ICP") {
        registration_ptr = std::make_shared<ICPRegistration>(config_node[registration_method]);
    } else if (registration_method == "ICP_SVD") {
        // TODO: register your custom implementation HERE:
        registration_ptr = std::make_shared<ICPSVDRegistration>(config_node[registration_method]);
    } else {
        LOG(ERROR) << "Point cloud registration method " << registration_method << " NOT FOUND!";
        return false;
    }
```

随后, 需要

* 在[此处](src/lidar_localization/include/lidar_localization/models/registration/icp_svd_registration.hpp), 增加里程计算法的**接口定义**
* 在[此处](src/lidar_localization/src/models/registration/icp_svd_registration.cpp#L162), 增加里程计算法的**实现**
    ```c++
    void ICPSVDRegistration::GetTransform(
        const std::vector<Eigen::Vector3f> &xs,
        const std::vector<Eigen::Vector3f> &ys,
        Eigen::Matrix4f &transformation_
    ) {
        const size_t N = xs.size();

        // TODO -- find centroids of mu_x and mu_y:

        // TODO -- build H:

        // TODO -- solve R:

        // TODO -- solve t:

        // set output:
        transformation_.setIdentity();
    }
    ```

在实现完成后

* 在Package [CMakeLists.txt](src/lidar_localization/CMakeLists.txt)中, 增加Build Config
* 将[config.yaml](src/lidar_localization/config/front_end/config.yaml#L1)中的算法配置改为`ICP_SVD`

    ```yaml
    # 匹配
    registration_method: ICP_SVD   # 选择点云匹配方法，目前支持：ICP, NDT, ICP_SVD
    ```

编译, 运行, 获取导出的轨迹, 使用evo进行精度评估.