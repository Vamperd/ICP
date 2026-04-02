# ICP 代码实现方法详细解读

这里为您详细解读 `icp_demo.m` 与 `icp_mapping.m` 这两个MATLAB脚本的实现逻辑和核心代码。这两个脚本主要基于**迭代最近点（Iterative Closest Point, ICP）**算法进行点云配准和三维建图。

---

## 1. `icp_demo.m` - 基础的两帧点云配准演示

这个脚本的主要作用是向用户展示如何用 ICP 算法对两帧点云数据进行匹配和对齐。

### 代码执行流程解析：

```matlab
ply_0 = pcread('ICP_data/0.ply');
ply_1 = pcread('ICP_data/1.ply');
```
* **读取数据**：使用 MATLAB 内置函数 `pcread` 从硬盘读取两帧点云数据（分别为第0帧和第1帧），其格式为常用的点云存储格式 `.ply`。`ply_0` 通常被视为目标点云（Target），`ply_1` 视为源点云（Source）。

```matlab
figure;
pcshowpair(ply_0, ply_1, 'MarkerSize', 50);
```
* **配准前可视化**：创建一个新图形窗口，使用 `pcshowpair` 叠加显示配准前两帧点云的相对位置，`MarkerSize` 设置为 50 以便清楚观察。可以直观看到未配准时两帧点云存在明显的平移和旋转偏差。

```matlab
[tform, ply_1] = pcregistericp(ply_1, ply_0);
```
* **核心ICP配准**：调用 `pcregistericp` 进行 ICP 算法计算。
  * **输入**：`ply_1`（要变换的源点云），`ply_0`（作为参考的目标点云）。
  * **输出**：`tform`（齐次变换矩阵对象，包含使得 `ply_1` 对齐到 `ply_0` 所需的旋转和平移），`ply_1`（此时的输出是经过配准变换后的新点云）。

```matlab
figure;
pcshowpair(ply_0, ply_1, 'MarkerSize', 50);
```
* **配准后可视化**：再次使用 `pcshowpair` 显示配准后的结果。如果 ICP 执行成功，两帧点云在空间上的特征将会很好地重合和对应在一起。

---

## 2. `icp_mapping.m` - 多帧点云拼接与全局建图

这个脚本是对 `icp_demo.m` 的进阶应用，展示了如何通过 ICP 算法将序列帧点云（第1至9帧）连续配准到一个全局地图中，完成类似 SLAM（同步定位与建图）后端的简化版多帧建图任务。

### 代码执行流程解析：

```matlab
clear;clc;
laser_map = pcread('ICP_data/0.ply');
```
* **初始化地图**：清空工作区后，读取第0帧点云 `0.ply` 作为我们的全局初始地图 `laser_map`。

```matlab
tform_init = affine3d([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]);
robot_tf{1} = tform_init;
```
* **初始化位姿**：创建一个代表单位阵（无任何平移和旋转）的三维仿射变换对象 `tform_init`。设定机器人在第0帧时的位姿为初始位姿，存入元胞数组 `robot_tf`。

```matlab
for i = 1:1:9
```
* **循环遍历新帧**：从第1帧循环一直到第9帧。

```matlab
    % read
    str = ['ICP_data/' ,num2str(i) , '.ply'];  
    curr_ply = pcread(str);
```
* **加载当前帧**：在循环内部，根据迭代变量拼凑文件名，使用 `pcread` 读取当前帧点云 `curr_ply`。

```matlab
    % icp
    [tform_init, curr_ply] = pcregistericp(curr_ply, laser_map, 'Metric','pointToPoint', 'InitialTransform', tform_init, 'MaxIterations', 100, 'Tolerance', [0.01, 0.001]);
    robot_tf{i+1} = tform_init;
```
* **带有高级参数的ICP配准**：
  * 将 `curr_ply` 配准到逐渐变大的全局地图 `laser_map` 上。
  * `'Metric','pointToPoint'`：指定 ICP 误差度量方式为点到点（Point-to-Point）距离。
  * `'InitialTransform', tform_init`：由于点云序列是连续拍摄的，上一帧的位姿常常是这一帧很好的初始猜测（Initial Guess）。传入上一步得到的变换作为初始值，极大避免了 ICP 陷入局部最优，同时加速收敛。
  * `'MaxIterations', 100`：限制 ICP 最大迭代次数为 100 次以防死循环。
  * `'Tolerance', [0.01, 0.001]`：设定算法收敛的容忍度阈值来判断是否停止迭代，包含相对位移收敛和相对旋转收敛两项。
  * 将新解算出来的位姿保存在 `robot_tf` 中，用于下一帧的初始位姿猜测和轨迹记录。

```matlab
    % merge
    laser_map = pcmerge(laser_map, curr_ply, 0.01);
```
* **地图融合与降采样**：使用 `pcmerge` 将当前已经配准好位置的 `curr_ply` 并入全局地图 `laser_map` 中。
  * 最后一个参数 `0.01` 指的是网格滤波（Voxel Grid Filter）的体素大小（BoxSize/GridSize）。该操作在融合两帧点云时会顺带进行下采样（Downsampling），防止拼接过程中点云数据量呈指数级膨胀，保证内存不溢出并提升下次 ICP 操作的速度。

```matlab
figure;
pcshow(laser_map, 'MarkerSize', 20);
```
* **最终结果展示**：经过所有循环后，9次拼接结束，得到完整的全局点云环境地图。使用 `pcshow` 绘制整张大地图。

---

### 总结比较
* **`icp_demo.m`** 类似于两图静止比对，演示 ICP 黑盒接口的使用与作用，没有引入状态传递。
* **`icp_mapping.m`** 模拟了真实的机器人探索场景：逐步累积地图，使用了诸如**初始位姿猜测提取（Initial Transform）**和**体素下采样融合地图（pcmerge with step）**等在实际 3D SLAM 中必需的方法以兼顾精度与计算效率。