# INSystem

## 1. 介绍

<div align="center">
	<img src="https://raw.githubusercontent.com/sunshanlu/INSystem/devel/GINS.gif" alt="INSystem" width=600>
</div>

参考高翔的自动驾驶与机器人中的SLAM技术的第四章，使用ROS2实现基于图优化和预积分的惯性导航系统。
1. 对ROS2做了深度集成
2. 使用c++17标准。
3. 使用了较新版的Sophus和g2o库

## 2. 依赖

1. ROS2 Foxy
2. C++17
3. Ubuntu20.04
4. Eigen3
5. geometry_msgs
6. sensor_msgs
7. [Pangolin 0.9.1](https://github.com/stevenlovegrove/Pangolin/releases/tag/v0.9.1)
8. [g2o_20201223](https://github.com/RainerKuemmerle/g2o/releases/tag/20201223_git)
9. [Sophus 1.22.10](https://github.com/strasdat/Sophus/releases/tag/1.22.10)
10. yaml-cpp

针对Pangolin、DBoW3和g2o，需要去上述指定的地址下载源码，然后编译安装。
对于没有指定地址的三方库，可以使用如下命令安装：
```shell
sudo apt update
sudo apt install libeigen3-dev libyaml-cpp-dev ros-foxy-geometry-msgs ros-foxy-sensor-msgs
```

## 3. 编译

```shell
git clone https://github.com/sunshanlu/INSystem.git
cd ORB-SLAM2-ROS2
colcon build
```

## 4. 运行

根据需求，创建配置文件，并运行，配置文件的示例在config文件夹下。config.yaml文件的每一项都做了注释，可以参考。

### 4.1 数据集示例

```shell
source ./install/setup.bash
./install/pre_insystem/lib/pre_insystem/pre_insystem_dataset config_path data_path
```

1. `config_path`：代表配置文件路径，例如`./config/config.yaml`
2. `data_path`：代表数据集路径，例如`./data/10.txt`

### 4.2 ROS2示例
```shell
source ./install/setup.bash
ros2 run pre_insystem pre_insystem_ros2 --ros-args -p ConfigPath:={config_path}
```

在此文件夹中新开终端运行数据发布示例
```shell
source ./install/setup.bash
ros2 run data_publisher data_publisher_node --ros-args -p DataPath:={data_path}
```

1. `config_path`：代表配置文件路径，例如`./config/config.yaml`
2. `data_path`：代表数据集路径，例如`./data/10.txt`

## 5. ROS2话题声明

1. `pre_insystem/odom`：代表惯性导航系统轮速计，类型为`pre_interfaces::msg::WOdom`
2. `pre_insystem/imu`：代表惯性导航系统IMU，类型为`sensor_msgs::msg::Imu`
3. `pre_insystem/rtk`：代表惯性导航系统RTK，类型为`pre_interfaces::msg::RTK`
4. `pre_insystem/navstate`：代表惯性导航系统导航状态，类型为`pre_interfaces::msg::NavState`

上述话题的1、2、3部分都是惯性导航系统的接收部分，而4为惯性导航系统的发布部分，其中包含位姿信息和速度信息。详细的pre_interfaces定义见`./src/pre_interfaces/msg`文件夹