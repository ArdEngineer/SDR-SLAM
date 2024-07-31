# 教程链接合集

## ubuntu系统镜像下载地址

#### 清华大学开源镜像站地址
https://mirror.tuna.tsinghua.edu.cn/

#### 下载ubuntu系统镜像的流程

找到 'ubuntu-releases' -> 选择ubuntu发行版本，点击进去，以下载ubuntu 16.04为例

一般我们用的都是64位的机器，选择ubuntu-16.04.7-desktop-amd64.iso下载即可

下载以.iso后缀的文件才能在虚拟机软件上安装（如果不会的话建议CSDN......）

#### 我们的项目优先推荐使用ubuntu 16.04 LTS，ubuntu 20.04 LTS也行

## ros系统安装教程

#### ubuntu 16.04 LTS + ros-kinetic安装教程
https://blog.csdn.net/chen20170325/article/details/120853610

#### ubuntu 20.04 LTS + ros-noetic安装教程
https://blog.csdn.net/qq_64671439/article/details/135287166

## 说明：我们的项目优先推荐使用ubuntu 16.04 LTS + ros-kinetic的组合

这样可以减少同一个工程项目在不同平台的部署和运行问题，如果大家熟悉跨平台C++工程部署的话

这话当我没说，因为SLAM很多代码对C++版本有要求，如果熟悉CMake指定C++编译版本的话可以自己随便配置运行环境

后面我会统一一下大家的环境部署要求，以方便大家同时合作开发和代码合并

## RoboWare ros开发工具安装说明（推荐的工具，装不装看自己）

这个部分可以直接到网上找教程，RoboWare的安装包我也放在资源文件夹那边了

这个东西是专门用来开发的ros的IDE，界面酷炫装逼，和VS Code相近，用起来挺方便的

## Euroc数据集下载地址

Euroc数据集太TM大了，十几个G，git push不了到仓库上面，这边建议各位亲自己下载...

https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

Euroc数据集有两种形式：rosbag包和MAV (ASL Data)，两种都可以，关于数据集的使用方式

我在数据集资源文件夹里面写了，大家自己看着捣鼓捣鼓......(MAV数据集会方便一些)

## OpenCV库下载

下载OpenCV需要使用梯子哦~

https://opencv.org/releases/

我们的项目使用的OpenCV版本请参考《视觉SLAM十四讲》里面的

另外双目相机可能使用OpenCV-3.4.3

## OpenCV的安装教程

由于不同版本的ubuntu安装OpenCV以及各自电脑上安装的依赖包有差异

所以这里建议大家具体问题具体分析，直接寻找符合自己环境的安装教程

## VINS-Mono + euroc数据集运行教程

https://blog.csdn.net/weixin_44417938/article/details/107294330

## VINS-Fusion + euroc数据集运行教程

https://blog.csdn.net/weixin_49247766/article/details/133694219

## 线性代数Gilbert Strang教材资源

我放在资源文件夹里面的是中文翻译版的，还有习题解析（纯英文版）