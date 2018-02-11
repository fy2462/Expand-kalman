# Expand-kalman

在一般的状态估计中，都存在两个方程，运动方程和观测方程。大体上讲，在评估位姿过程中，分为两类方法：
 1. 假设k时刻只与k-1时刻相关，那么即为kalam方向，它是一种马尔可夫性质推测，根据概率模型推测下一阶段的最大化的位姿状态。
 2. 考虑k时刻和这之前的所有状态关系，此时往往状态之间是非线性关系，那么既为最小优化方向，可通过构造最小二乘方程，求解优化后的整体位姿。
  
 本文实现了C++编写的kalman filter代码，其中输入数据为雷达和激光雷达数据，输出为预测物体的位姿、速度等数据。
 
## 原理

### 计算步骤
卡尔曼滤波算法遵循如下的计算步骤
1. 初始化：接收第一次测量的数据并初始化转换矩阵、协方差矩阵等初始变量。
2. 预测：通过初始化数据和协方差，计算Δt时间后的新位置和方差。
3. 更新：预测数据与真实数据进行比较，得出下一个次循环所需的协方差矩阵等变量，重复2-3即可持续工作。
   
### 计算流程

![image](https://github.com/fy2462/Expand-kalman/blob/master/image/prediction-update.jpg)

### 变量定义
**x** 是状态向量。包含追踪对象的位置和速度信息，服从高斯分布。

**P** 是状态协方差矩阵。量化当前追踪对象的位置和速度的不确定性。

**k** 标识时间步长。表示在时间k的对象状态数据。

**k+1|k** 先验预测步长。根据k时的先验数据，预测k+1时的位姿（此时未参考传感器真实测量数据）

* **Prediction**

![image](https://github.com/fy2462/Expand-kalman/blob/master/image/kalman-prediction.jpg)

    **x'** 预测后的状态向量分布均值。
    
    **P'** 预测后的状态向量协方差矩阵。
    
    ```
    假设我们知道一个对象的当前位置和速度为x。 我们可以预测物体在Δt之后的状态，因为我们知道Δt前的物体位置和速度，我们可以假设物体保持相同的速度在运动。 
    利用x'= Fx +ν函数做预测计算。 但也许对象没有保持完全相同的速度，也许物体改变方向，加速或减速。 所以当我们预测Δt后的状态时，状态P的不确定性就会增加为P'。
    由于实际物体可能会有加速、减速，所以P'服从ν〜N（0，Q）的高斯分布，以描述这种预测的不确定性。
    ```
* **Update**

![image](https://github.com/fy2462/Expand-kalman/blob/master/image/kalman-update.jpg)

    ```
    在得到带有误差的传感器数据z后，首先我们和预测结果x'进行对比y = z-Hx', 其中H为线性转换矩阵。
    定义K矩阵（卡尔曼滤波器增益）将我们预测的P'的不确定度与传感器测量R的不确定度相结合，测量噪声服从ω〜N（0，R）的高斯分布。
    如果我们的传感器测量非常不确定（R相对于P'高） 那么卡尔曼滤波器会侧重于我们的预测结果:x'。 
    如果我们的预测是更加不确定（P'相对于R高），卡尔曼滤波器将会对传感器测量的结果z更加重视。
    ```
    
### 扩展kalman
有时我们采集的数据来自多种传感器，其中有些传感器数据和噪声并不服从高斯分布，由于高斯分布经过非线性变换后，不再是高斯分布，所以到时上述的kalman方程无法递推下去。
于是EKF使用高斯分布的近似去模拟非线性数据，并在k时刻进行线性计算。

![image](https://github.com/fy2462/Expand-kalman/blob/master/image/ekf-eqution.jpg)

这是在k处的1阶泰勒展开，以近似模拟k-1到k的非线性变化。
下图为扩展kalman与传统kalman在计算上的区别：

![image](https://github.com/fy2462/Expand-kalman/blob/master/image/efk.jpg)

其中：**x'=f(x,u)** 为k-1出的先验数据(u=0)，h(x')为非线性转换函数转换后的预测值。Hj为雅可比矩阵，是一阶泰勒展开的系数矩阵。

## 实际数据计算

### 激光雷达数据
激光雷达返回的数据通常是点云数据(x,y,z)，本示例所输入的激光数据是经过转换后的物体中心坐标和速度2维向量等物体运动状态。
所以我们建立预测数据等式：

![image](https://github.com/fy2462/Expand-kalman/blob/master/image/linaer-conversation.jpg)

```$xslt
其中预测噪声示例中忽略为0，读者可以后续添加高斯随机噪声。
```
同时运动状态方程的噪声协方差为：

![image](https://github.com/fy2462/Expand-kalman/blob/master/image/gaussian-distribution.jpg)

随后通过计算kalman增益K, 对预测的状态向量进行调整，得出最终的预测xk和Pk。

### 无线雷达数据

与激光雷达相比，雷达的空间分辨率更低，但是雷达能够提供径向速度信息。因此我们通常需要聚合激光雷达与雷达数据。
但是雷达数据往往只会得到距离和角度，并且与激光雷达数据是无法线性转换的，所以这就需要设计一个新的卡尔曼滤波方程。
雷达返回的数据如下所示：

![image](https://github.com/fy2462/Expand-kalman/blob/master/image/radar-data.jpg)

**ρ** 与观测障碍物的距离。

**φ** 与观测障碍物的角度。

**ρ˙** 为障碍物速度在观察方向的投影，也就是径向速度。

我们需要将先验的数据先从极坐标转换为笛卡尔坐标。

![image](https://github.com/fy2462/Expand-kalman/blob/master/image/hx.jpg)

观察发现，这个转换过程并不是一个线性转换。然后我们计算雅可比矩阵Hj：

![image](https://github.com/fy2462/Expand-kalman/blob/master/image/jacobian.jpg)

其余与传统kalman过程类似，最终我们得到k时刻的xk和Pk。

## 数据文件描述

## 代码简述

### 代码依赖