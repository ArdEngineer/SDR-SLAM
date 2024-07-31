# 教程链接合集

## ubuntu系统镜像下载地址

#### 清华大学开源镜像站地址
https://mirror.tuna.tsinghua.edu.cn/

#### 下载ubuntu系统镜像的流程

找到 'ubuntu-releases' -> 选择ubuntu发行版本，点击进去，以下载ubuntu 16.04为例

一般我们用的都是64位的机器，选择ubuntu-16.04.7-desktop-amd64.iso下载即可

下载以.iso后缀的文件才能在虚拟机软件上安装（如果不会的话建议CSDN......）

### 我们的项目优先推荐使用ubuntu 16.04 LTS + ros-kinetic的环境配置

## ros系统安装教程

#### ubuntu 16.04 LTS + ros-kinetic安装教程
https://blog.csdn.net/chen20170325/article/details/120853610

#### ubuntu 20.04 LTS + ros-noetic安装教程
https://blog.csdn.net/qq_64671439/article/details/135287166

## 说明：我们的项目优先推荐使用ubuntu 16.04 LTS + ros-kinetic的组合

这样可以减少同一个工程项目在不同平台的部署和运行问题，如果大家熟悉跨平台C++工程部署的话,

这话当我没说，因为SLAM很多代码对C++版本有要求，如果熟悉CMake指定C++编译版本的话可以自己随便配置运行环境

后面我会统一一下大家的环境部署要求，以方便大家同时合作开发和代码合并

# 下面的教程没有特别说明的话，默认都是在ubuntu 16.04 LTS + ros-kinetic上运行的

涉及到非ubuntu16.04 LTS + ros-kinetic的环境的教程，我会抽时间找一下

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

另外双目相机INDEMIND stereo camera可能会使用到OpenCV-3.4.3

以及VINS-Mono和VINS-Fusion都是依赖于OpenCV 3系列运行的

如果你装的是Ubuntu 20.04 + ros-noetic的环境的话，这个环境默认是OpenCV 4系列

到时候你还得自己把VINS-Mono/Fusion代码改一遍还要自己装一遍OpenCV还要处理多版本OpenCV共存问题......麻烦得要死

所以这就是为什么我推荐ubuntu 16.04 LTS + ros-kinetic，两个字：省事，一遍就搞定

## OpenCV的安装教程

由于不同版本的ubuntu安装OpenCV以及各自电脑上安装的依赖包有差异

所以这里建议大家具体问题具体分析，直接寻找符合自己环境的安装教程

## VINS-Mono + euroc数据集运行教程

https://blog.csdn.net/weixin_44417938/article/details/107294330

## VINS-Fusion + euroc数据集运行教程

https://blog.csdn.net/weixin_49247766/article/details/133694219

## ubuntu 20.04 LTS + ros-noetic + euroc数据集运行VINS-Fusion教程

https://blog.csdn.net/m0_37993445/article/details/123075977

## INDEMIND双目相机SDK安装和相机测试教程

ubuntu安装SDk和相机测试官方教程（我的评价是：史

https://imsee-sdk-docs.readthedocs.io/zh/latest/src/sdk/install_ubuntu.html

ros-wrapper安装和相机测试教程（推荐直接测试这个，第一个就是一坨屎）

依赖包安装参考

https://blog.csdn.net/Prototype___/article/details/129717553

相机测试参考：

https://imsee-sdk-docs.readthedocs.io/zh/latest/src/sdk/install_ros_wrapper.html

## 线性代数Gilbert Strang教材资源

我放在资源文件夹里面的是中文翻译版的，还有习题解析（纯英文版）

## ROS系统教程

这里推荐机械工业出版社《ROS机器人开发实践》（胡春旭）这一本教程

了解ros话题通信机制，了解ros系统的结构，以及了解ros bag包的功能和作用

再有就是学习一下ros里面关于集成SLAM的一些rosnode的使用方法即可