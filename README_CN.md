# KF-GINS

[[中]](./README_CN.md) &ensp; [[EN]](./README.md)

## 基于扩展卡尔曼滤波的GNSS/INS组合导航软件

## 简介
我们开源了一套基于扩展卡尔曼滤波的GNSS/INS组合导航软件KF-GINS[^1]。KF-GINS实现经典的GNSS位置和IMU数据的组合导航解算，算法实现参考牛小骥教授和陈起金博士的《惯性导航原理与GNSS/INS组合导航》课程讲义，作为课程的配套资源。软件采用C++语言编写，采用CMake管理项目。
KF-GINS的主要特点有：

- 扩展卡尔曼滤波架构(误差状态向量)的GNSS/INS松组合算法，包括IMU误差补偿、惯性导航解算、Kalman滤波、误差反馈等环节
- 采用21维系统误差状态，包括位置误差、速度误差、姿态误差、IMU零偏误差和IMU比例因子误差
- 姿态误差采用Phi角模型，速度、位置误差定义在导航坐标系下
- 惯性导航解算采用基于线性变化假设的双子样机械编排算法，补偿了姿态圆锥效应、速度的旋转效应和划桨效应


**单位:** 武汉大学卫星导航定位技术研究中心[多源智能导航实验室](http://www.i2nav.com/)

**相关资料:**

- 牛小骥, 陈起金, "[惯性导航原理与GNSS/INS组合导航课程讲义](http://www.i2nav.com/index/newListDetail_zw.do?newskind_id=13a8654e060c40c69e5f3d4c13069078&newsinfo_id=40f3c65b158742c099ba3b600c983aa1)", 武汉大学多源智能导航实验室, 2022
- 牛小骥, 陈起金, "[惯性导航原理与GNSS/INS组合导航课程视频](https://www.bilibili.com/video/BV1na411Z7rQ?spm_id_from=333.999.0.0&vd_source=a417ebe0768fc96919fe8e34c55ed591)", 武汉大学多源智能导航实验室， 2022
- X. Niu, Q. Zhang, L. Gong, C. Liu, H. Zhang, C. Shi, J. Wang and M. Coleman (2014). "Development and evaluation of GNSS/INS data processing software for position and orientation systems." Survey Review 2014; 47(341), 87-98.
- 严恭敏, 翁浚, 捷联惯导算法与组合导航原理. 西北工业大学出版社, 2019.
- E.-H. Shin, "Estimation techniques for low-cost inertial navigation," Ph.D. dissertation, Dept. Geomatics Eng., Univ. of Calary, AB, Cabada, 2005
- [Savage, P.G.](http://www.strapdownassociates.com/), "Strapdown analytics, Part I", Maple Plain, MN: Strapdown Associates, 2000.
- Titterton David, JohnL. Weston, and John Weston, "Strapdown inertial navigation technology", The Institution of Electrical Engineers, 2004.
- P. D. Groves, "Principles of GNSS, Inertial, and Multi-sensor Integrated Navigation Systems", 2nd ed., vol. 39. Artech House, 2013.

**如果你使用这个软件进行学术研究，请添加如下致谢，并引用我们的[相关文档和论文](./ref.bib)**

```
中文模板：“本文作者感谢武汉大学卫星导航定位技术研究中心多源智能导航实验室(i2Nav)牛小骥教授团队开源的KF-GINS软件平台。”
English version: “The authors would like to acknowledge the team of Prof. Xiaoji Niu of the Integrated and Intelligent Navigation (i2Nav) group from GNSS Research Center of Wuhan University for providing the open-source KF-GINS software that was used in the paper.”
```

**联系:**

- 有任何相关的技术问题，可以向王立强(wlq@whu.edu.cn)或唐海亮博士(thl@whu.edu.cn)发送邮件。
- 中国使用者可以加入QQ交流群(481173293)进行讨论，进群需要提供所在机构和真实姓名。

[^1]: 我们也开源了一套基于图优化的GNSS/INS组合导航算法 [OB_GINS](https://github.com/i2Nav-WHU/OB_GINS).

## 1.程序编译与运行
### 1.1 编译环境

KF-GINS项目使用CMake管理，支持在Linux环境，MacOS环境和Windows环境下编译。我们建议优先选择Linux环境进行编译。

KF-GINS编译成功后需要使用配置文件作为参数。程序调试时也需要添加命令行参数。

### 1.2 依赖库
除了基本的C++标准库之外，KF-GINS依赖三个库，分别为Eigen3，abseil-cpp和yaml-cpp. 这三个库已经作为三方库加到工程源代码中，不需要使用者单独安装。

### 1.3 在 Linux内编译

我们建议使用Ubuntu18.04或者Ubuntu20.04系统下g++编译工具进行KF-GINS编译，编译前需安装必要的库：
```shell
sudo apt-get install cmake
sudo apt-get install build-essential
```

配置好自己的编译环境之后，将仓库克隆到本地后并按照如下操作编译KF-GINS：

```shell
# 克隆仓库
git clone https://github.com/i2Nav-WHU/KF-GINS.git ~/

# 编译KF-GINS
cd ~/KF-GINS
mkdir build && cd build

cmake ../ -DCMAKE_BUILD_TYPE=Release 
make -j8

# 运行测试数据集
cd ~/KF-GINS
./bin/KF-GINS ./dataset/kf-gins.yaml

# 等待程序运行结束
```

### 1.4 在 Windows 内编译

Windows环境下一般使用MSVC(Microsoft Visual C/C++)编译器，我们建议在VSCode软件下进行编译。

Windows下需要通过Visual Studio Installer安装MSVC编译器；下载安装VSCode软件，并安装必要的插件，包括C/C++、C/C++ Extension Pack、CMake、CMake Tools.

克隆仓库到本地，并通过VSCode打开仓库文件夹。

- 选择编译器，打开命令面板(Ctrl+Shift+P)输入"CMake: Select a Kit"，选择MSVC编译器
- 设置编译参数，命令面板输入"CMake: Select Variant"，选择Release
- 配置CMake，命令面板输入"CMake: Configure" 
- 编译工程，命令面板输入"CMake: Build" 

在工程目录下打开PowerShell或CMD终端，运行测试数据集：
```shell
.\bin\KF-GINS.exe .\dataset\kf-gins.yaml
# 可执行文件可能在 .\bin\Release 文件夹下生成，则执行命令为:
# .\bin\Release\KF-GINS.exe .\dataset\kf-gins.yaml
```

### 1.5 在 MacOS内编译

在 MacOS 中使用 xcode-select 和 cmake。编译前需安装必要的工具：

```shell
xcode-select --install
brew install cmake
```

配置好自己的编译环境之后，将仓库克隆到本地后并按照如下操作编译KF-GINS：

```shell
# 克隆仓库
git clone https://github.com/i2Nav-WHU/KF-GINS.git ~/

# 编译KF-GINS
cd ~/KF-GINS
mkdir build && cd build

cmake ../ -DCMAKE_BUILD_TYPE=Release 
make -j8

# 运行测试数据集
cd ~/KF-GINS
./bin/KF-GINS ./dataset/kf-gins.yaml

# 等待程序运行结束
```

### 1.6 在 VSCode 内调试

Linux 下调试需要安装gdb调试工具：
```shell
sudo apt-get install gdb
```

调试操作如下：

- 选择编译器，打开命令面板(Ctrl+Shift+P)输入"CMake: Select a Kit"，Linux选择gcc编辑器，Windows选择MSVC编译器
- 设置编译参数，命令面板输入"CMake: Select Variant"，选择 **Debug**
- 配置CMake，命令面板输入"CMake: Configure"，编译工程，命令面板输入"CMake: Build"
- 选择调试启动选项，点击VSCode左侧第四个按钮打开“运行和调试”菜单(或快捷键Ctrl+Shift+D)，左侧窗口上方选择“Windows 启动” 或 “Linux gdb 启动”
- 开始调试，点击左侧窗口上方的 绿色三角符号 开始调试(或快捷键F5调试)

## 2 KF-GINS使用

### 2.1 基础知识

**坐标系定义：**
- IMU坐标系：原点为IMU测量位置，轴向为前-右-下方向
- 导航参考坐标系：原点和IMU坐标系一致，轴向为北-东-地方向

**导航状态：**
- 位置：IMU在地球坐标系下的大地位置 (纬度-经度-椭球高)
- 速度：IMU相对于地球的速度投影在导航参考坐标系下 (北向速度-东向速度-垂向速度)
- 姿态：IMU相对于导航参考坐标系的姿态角 (四元数、方向余弦矩阵或欧拉角，其中欧拉角定义为航向角-俯仰角-横滚角，ZYX旋转顺序)

**IMU误差建模：**
- IMU测量噪声建模为高斯白噪声
- IMU零偏误差建模为一阶高斯马尔科夫过程
- IMU比例因子误差建模为一阶高斯马尔科夫过程

更多算法细节请参考[惯性导航原理与GNSS/INS组合导航课程讲义](http://www.i2nav.com/index/newListDetail_zw.do?newskind_id=13a8654e060c40c69e5f3d4c13069078&newsinfo_id=40f3c65b158742c099ba3b600c983aa1).

### 2.2 数据格式

- IMU文本文件格式定义为:

| 列数 | 数据描述         | 单位  |
| ---- | ---------------- | ----- |
| 1    | GNSS 周内秒      | $s$   |
| 2~4  | X-Y-Z轴 角度增量 | $rad$ |
| 5~7  | X-Y-Z轴 速度增量 | $m/s$ |

- GNSS位置文本文件格式定义为:

| 列数 | 数据描述                    | 单位  |
| ---- | --------------------------- | ----- |
| 1    | GNSS 周内秒                 | $s$   |
| 2    | 纬度                        | $deg$ |
| 3    | 经度                        | $deg$ |
| 4    | 椭球高                      | $m$   |
| 5~7  | 位置标准差 (北向-东向-地向) | $m$   |

- 导航结果和参考真值文本文件格式定义为:

| 列数 | 数据描述                | 单位  |
| ---- | ----------------------- | ----- |
| 1    | GNSS 周                 | -     |
| 2    | GNSS 周内秒             | $s$   |
| 3    | 纬度                    | $deg$ |
| 4    | 经度                    | $deg$ |
| 5    | 椭球高                  | $m$   |
| 6~8  | 三维速度 (北向-东向-地向)     | $m/s$ |
| 9~11 | 姿态角 (横滚-俯仰-航向) | $deg$ |

- IMU误差文件二进制文件格式(均为double类型)定义为:

| 列数  | 数据描述                 | 单位    |
| ----- | ------------------------ | ------- |
| 1     | GNSS 周内秒              | $s$     |
| 2~4   | X-Y-Z轴 陀螺仪零偏       | $deg/h$ |
| 5~7   | X-Y-Z轴 加速度计零偏     | $mGal$  |
| 8~10  | X-Y-Z轴 陀螺仪比例因子   | $ppm$   |
| 11~13 | X-Y-Z轴 加速度计比例因子 | $ppm$   |

- 系统状态标准差二进制文件格式(均为double类型)定义为:

| 列数  | 数据描述               | 单位    |
| ----- |--------------------| ------- |
| 1     | GNSS 周内秒           | $s$     |
| 2~4   | 三维位置标准差 (北向-东向-地向)    | $m$     |
| 5~7   | 三维速度标准差 (北向-东向-地向)    | $m/s$   |
| 8~10  | 三维姿态标准差 (横滚-俯仰-航向) | $deg$   |
| 11~13 | X-Y-Z轴 陀螺仪零偏标准差    | $deg/h$ |
| 14~16 | X-Y-Z轴 加速度计零偏标准差   | $mGal$  |
| 17~19 | X-Y-Z轴 陀螺仪比例因子标准差  | $ppm$   |
| 20~22 | X-Y-Z轴 加速度计比例因子标准差 | $ppm$   |


### 2.3 初始对准

KF-GINS目前只支持给定全部初始状态进行初始对准, 程序运行前需要在配置文件(kf-gins.yaml)中配置初始信息。

## 3 数据集

### 3.1 测试数据

我们提供了一组测试数据并给定了配置文件，保存在**dataset**文件夹中。

### 3.2 开源数据集

使用者可以下载我们的开源数据集 **[awesome-gins-datasets](https://github.com/i2Nav-WHU/awesome-gins-datasets)** 进行测试。

### 3.3 使用者的数据

KF-GINS使用的数据格式在 **[awesome-gins-datasets](https://github.com/i2Nav-WHU/awesome-gins-datasets)** 数据集中有详细定义，使用者可以参考数据格式定义自己的数据。


## 4 许可

KF-GINS 源代码在 GPLv3 许可下发布。

我们仍在努力提高代码的可靠性。如有任何技术问题，请联系王立强(wlq@whu.edu.cn)或唐海亮(thl@whu.edu.cn)，或在仓库中开启一个议题。

商业用途请联系牛小骥教授(xjniu@whu.edu.cn)。

