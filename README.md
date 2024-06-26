# 远程火箭弹道仿真

本作业题目为2024春季学期，其中题目如下：

## 参数

### 总体参数

| 参数 | 值 |
|------|-----|
| 级数 | 3 |
| 各级起飞质量(kg) | 18800, 5400, 1300 |
| 各级平均直径(m) | 2.0, 2.0, 1.5 |
| 各级平均推力(kN) | 485, 120, 35 |
| 各级比冲(s) | 240, 245, 255 |
| 各级工作时间(s) | 72, 71, 64 |

秒耗量根据推力和比冲计算，计算秒耗量时可采用常值g=9.806 m/s2。

### 飞行程序

俯仰角见附件。侧滑角和滚转角为零。

### 气动参数

- 高度0-40km
  - 阻力系数：0.09，升力系数0.13；
- 高度40-60km
  - 阻力系数：0.04，升力系数0.08；
- 高度60km 以上
  - 阻力系数：0.00，升力系数0.00。

### 其它参数

| 项目 | 数值 | 单位 |
| --- | --- | --- |
| 地球平均半径 | 6371004 | m |
| J2 项 | 1.082e-3 | 无单位 |
| 地球扁率 | 1/298.257 | 无单位 |
| 地球引力常量 | 3.986e14 | m²/s³ |
| 地球引力加速度 | 9.806 | m/s² |
| 地球自转角速度 | 7.292e-5 | rad/s |
| 发射点经度 | E 60° | 无单位 |
| 发射点纬度 | S 30° | 无单位 |
| 发射点高度 | 0 | m |
| 发射方位角 | -20° | 无单位 |
| 地表大气密度 | 1.225 | kg/m³ |

大气密度可按照指数模型

## 代码任务

- 计算**主动段弹道**，给出包括`高度`、`速度`、`弹道倾角`、`动压`、`过载`、`攻角`、
`俯仰角`、`质量`、`经纬度`、`坐标`(如发射系下的x, y, vx, vy)等数据的图像

- 给出**一级飞行**时`最大攻角`、`最大动压`、`最大法向过载`

- 采用**速度坐标系**下动力学方程解算弹道

## 代码编写

|-Rocket
    |-Rocket
|-
