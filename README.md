# 这是什么

这是一个标定激光雷达和旋转电机安装偏差的项目, 具体偏差请看下面图示  
其中 **L** 为激光雷达坐标系, **M** 为电机坐标系, 二者在理想状态下应完全重合, 这样后续拼装数据只需要考虑激光雷达跟随电机的朝向, 但是因为这个安装误差的存在会导致在只考虑电机朝向时原数据出现畸变, 即无法正确还原原始场景
![](./README.d/外旋转激光雷达外参.png)

# 项目架构

- 库
  - preprocessor: 一些预处理工具, 包括 io, 拼装, 提取特征等
  - calibrator: 外参矫正算法, 设置好各种配置后直接调用 `run(ans)` 就能得到标定结果
  - utils: 配合 yaml-cpp 读写配置文件
- 可执行文件
  - cmb: 不考虑外部参数的情况下只根据电机朝向组装数据, 从 `raw_dirname` 加载数据, 将组装好的数据保存到`cmb_filename`
  - calib: 根据提取出来的特征进行标定, 读取 `raw_dirname` 和 `obj_dirname`, 标定结果写入 `mat_file_name`
  - apb: 根据 `mat_file_name` 中的外参组装 `raw_dirname` 中的原始数据, 并保存到 `cmb_filename`, 供可视化检验

# 算法

算法参考了以下论文

- J. Kang 和 N. L. Doh, 《Full-DOF Calibration of a Rotating 2-D LIDAR With a Simple Plane Measurement》, IEEE Trans. Robot., 卷 32, 期 5, 页 1245–1263, 10 月 2016, doi: 10.1109/TRO.2016.2596769.

# 使用方法

## 拉取代码

```bash
git clone ...

```

## 构建项目

```bash
cmake -D CMAKE_BUILD_TYPE=Release -S . -B build
cmake --build build
```

## 使用

**所有数据保存在 `data` 文件夹**  
**所有配置文件与外参保存在 `cfg` 文件夹**

1. 使用 `caliba_cmb` 将原始数据组装在一起
2. 使用任何一种点云可视化软件将组装结果中面积较大, 点数较多的平面分割出来, 分别保存到不同的文件中, 支持 pcd ply 两种格式
   如果使用 `cloudcompare` 注意保存后的结果中 scalar 部分, 软件会自动添加 \_scalar\_\_ 前缀, 需要删掉
3. 使用 `caliba_clb` 进行外参标定
4. 如果不确定标定效, 可使用 `caliba_apb` 对原始数据应用标定结果, 用可视化软件查看标定结果
