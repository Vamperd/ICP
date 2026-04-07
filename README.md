# IPC 二维点云匹配作业说明

本仓库用于完成二维激光点云配准作业，包含：

- MATLAB 内置 ICP 基准脚本
- 自实现原始 ICP
- 自实现 PL-ICP
- 自实现 N-ICP
- 结果图与 `.mat` 数据保存逻辑

本文档重点说明：

1. 仓库结构
2. 每个脚本的作用
3. 如何运行程序
4. 程序会生成哪些结果
5. 这些结果在实验报告中分别有什么作用

---

## 1. 仓库结构

根目录主要文件如下：

- [ICP_data](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/ICP_data)
  - 实验输入数据，包含 `0.ply ~ 9.ply`
- [icp_demo.m](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/icp_demo.m)
  - MATLAB 内置 ICP 的双帧示例
- [icp_mapping.m](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/icp_mapping.m)
  - MATLAB 内置 ICP 的顺序建图基准
- [myicp.m](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/myicp.m)
  - 自实现原始点到点 ICP
- [myplicp.m](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/myplicp.m)
  - 自实现点到线 ICP
- [mynicp.m](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/mynicp.m)
  - 自实现法向增强点到线 ICP
- [myicp_mapping.m](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/myicp_mapping.m)
  - 原始 ICP 主程序
- [myplicp_mapping.m](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/myplicp_mapping.m)
  - PL-ICP 主程序
- [mynicp_mapping.m](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/mynicp_mapping.m)
  - N-ICP 主程序
- [explain.md](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/explain.md)
  - 算法详细说明文档
- [results_matlab](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/results_matlab)
  - 所有结果图和结果数据的输出目录
- [SYK_ICP](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/SYK_ICP)
  - 参考实现目录，用于对照思路与效果

---

## 2. 各脚本的作用

### 2.1 基准脚本

#### `icp_demo.m`

作用：

- 使用 MATLAB 内置 `pcregistericp`
- 对 `0.ply` 与 `1.ply` 做双帧配准
- 适合演示最基础的 ICP 配准过程

#### `icp_mapping.m`

作用：

- 使用 MATLAB 内置 `pcregistericp`
- 对 `0.ply ~ 9.ply` 做顺序建图
- 作为 baseline 对照图使用

### 2.2 自实现脚本

#### `myicp.m`

作用：

- 自实现原始点到点 ICP
- 最近邻匹配 + SVD 刚体求解

#### `myplicp.m`

作用：

- 自实现点到线 ICP
- 当前点先与目标地图局部线段建立关系，再投影到线段上

#### `mynicp.m`

作用：

- 自实现法向引导的点到线 ICP
- 在 PL-ICP 基础上增加法向一致性、多候选锚点和切向位移约束

### 2.3 三个主程序

#### `myicp_mapping.m`

作用：

- 顺序运行原始 ICP
- 输出原始 ICP 的地图、轨迹图和误差图

#### `myplicp_mapping.m`

作用：

- 顺序运行 PL-ICP
- 输出 PL-ICP 的地图、轨迹图和误差图

#### `mynicp_mapping.m`

作用：

- 顺序运行 N-ICP
- 输出 N-ICP 的地图、轨迹图和误差图

---

## 3. 运行环境

本项目使用 MATLAB 运行。

要求：

- MATLAB 已安装
- 当前工作目录为仓库根目录
- `ICP_data` 文件夹存在且包含 `0.ply ~ 9.ply`

---

## 4. 如何运行

### 4.1 运行内置双帧 ICP 示例

```matlab
run('icp_demo.m')
```

作用：

- 展示 MATLAB 内置 ICP 对两帧点云的配准效果

### 4.2 运行内置顺序建图基准

```matlab
run('icp_mapping.m')
```

作用：

- 生成 MATLAB 内置 ICP 的 baseline 地图

输出：

- [results_matlab/icp_baseline_map.png](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/results_matlab/icp_baseline_map.png)

### 4.3 运行自实现原始 ICP

```matlab
run('myicp_mapping.m')
```

### 4.4 运行自实现 PL-ICP

```matlab
run('myplicp_mapping.m')
```

### 4.5 运行自实现 N-ICP

```matlab
run('mynicp_mapping.m')
```

### 4.6 命令行方式运行

如果希望从终端运行，可使用：

```powershell
matlab -batch "run('myicp_mapping.m')"
matlab -batch "run('myplicp_mapping.m')"
matlab -batch "run('mynicp_mapping.m')"
```

---

## 5. 运行后会生成什么

每个自实现算法都会在 `results_matlab/<algorithm>/` 下生成 4 个文件。

### 5.1 原始 ICP 输出目录

目录：

- [results_matlab/icp](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/results_matlab/icp)

文件：

- `local_map.png`
- `trajectory_topdown.png`
- `mse_overview.png`
- `result_summary.mat`

### 5.2 PL-ICP 输出目录

目录：

- [results_matlab/plicp](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/results_matlab/plicp)

文件：

- `local_map.png`
- `trajectory_topdown.png`
- `mse_overview.png`
- `result_summary.mat`

### 5.3 N-ICP 输出目录

目录：

- [results_matlab/nicp](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/results_matlab/nicp)

文件：

- `local_map.png`
- `trajectory_topdown.png`
- `mse_overview.png`
- `result_summary.mat`

---

## 6. 每个输出文件的作用

### 6.1 `local_map.png`

作用：

- 展示最终累计地图
- 同时叠加轨迹

报告中可用于说明：

- 点云是否重合紧密
- 墙面是否清晰
- 整体建图是否成功
- 算法是否出现明显漂移

### 6.2 `trajectory_topdown.png`

作用：

- 展示 10 帧位姿的俯视轨迹

报告中可用于说明：

- 轨迹是否平滑
- 位姿是否连续
- 不同算法的漂移程度

### 6.3 `mse_overview.png`

作用：

- 用 3x3 子图展示 9 次配准的误差曲线

报告中可用于说明：

- 哪几对点云更难配准
- 收敛速度是否稳定
- 不同算法在误差下降速度上的差异

### 6.4 `result_summary.mat`

作用：

- 保存本次运行的核心数据

当前包含：

- `robot_tf`
  - 10 帧累计位姿矩阵
- `global_poses`
  - 全局位姿表
- `metrics_table`
  - 每对配准的指标表
- `MSE`
  - 每对配准的误差曲线

这个文件适合后续：

- 在 MATLAB 中继续画图
- 导出到报告表格
- 做算法对比统计

---

## 7. 结果表的含义

每个主程序运行结束后，MATLAB 控制台都会打印两张表。

### 7.1 `global_poses`

含义：

- 第 0 到第 9 帧在全局地图坐标系下的位置和朝向

字段：

- `global_x`
- `global_y`
- `global_theta`

适用：

- 说明轨迹走势
- 做位姿对比

### 7.2 `metrics_table`

含义：

- 每一对相邻帧配准完成后的关键指标

字段：

- `iteration_count`
  - 实际迭代次数
- `valid_correspondence_count`
  - 最终参与优化的有效匹配点数
- `mean_residual`
  - 当前实现中实际是 `RMSE = sqrt(MSE)`

注意：

- `mean_residual` 虽然名字叫 residual，但它不是简单平均距离
- 对于 PL-ICP 和 N-ICP，它更接近“投影匹配误差”的 RMSE
- 因此跨算法比较时不能只看这个数，必须结合地图和轨迹一起看

---

## 8. 当前推荐的主程序使用方式

如果实验报告需要展示三种算法的结果，推荐按照以下顺序运行：

1. `run('icp_mapping.m')`
2. `run('myicp_mapping.m')`
3. `run('myplicp_mapping.m')`
4. `run('mynicp_mapping.m')`

这样可以得到：

- MATLAB 内置 baseline
- 自实现原始 ICP
- 自实现 PL-ICP
- 自实现 N-ICP

便于直接横向比较。

---

## 9. 当前 N-ICP 调参结论

在当前版本中，`mynicp_mapping.m` 采用的是经过小范围扫描后选出的较稳参数：

- `dist_threshold = 1.0`
- `cos_threshold = 0.08`
- `tangent_threshold = 1.2`
- `candidate_k = 6`

选择依据：

- 保证最终轨迹长度足够接近有效建图尺度
- 同时避免出现明显退化或过早卡住

这组参数不是全局最优，只是当前实验条件下比较稳的一组。

---

## 10. 实验报告撰写建议

后续写实验报告时，建议按下面结构组织：

1. 任务背景与数据说明
2. MATLAB 内置 ICP baseline
3. 原始 ICP 原理与结果
4. PL-ICP 原理与结果
5. N-ICP 原理与结果
6. 三种方法的对比分析
7. 误差讨论与参数影响分析
8. 总结

其中可以直接引用：

- 算法原理：见 [explain.md](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/explain.md)
- 结果图片：见 [results_matlab](c:/Users/86136/Desktop/大三春夏/大三春 智能移动技术/IPC/results_matlab)
- 结果数据：见各目录下的 `result_summary.mat`

---

## 11. 进一步改进建议

如果后续还要继续优化实验效果，可以优先尝试：

- 给原始 ICP 增加距离门限和异常匹配剔除
- 给 PL-ICP 增加更严格的局部线段质量筛选
- 给 N-ICP 调整法向估计半径、候选数量和评分权重
- 在报告中加入不同参数下的轨迹与地图对比图

这样可以进一步增强实验分析的完整性和说服力。
