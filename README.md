<h1 align='center' style="text-align: center;">SDR-SLAM : A Resilient SLAM Framework</h1> 

<h3 align='center' style="text-align: center;">The project is still UNDER DEVELOPMENT.</h3>

<h2 align='center' style="text-align: center;"><img src="./icons/icon4.png" style="height: 25px; vertical-align: middle; margin-right: 1px;"> Hightlights of this VINS system</h2>

### <img src="./icons/icon3.png" style="height: 20px; vertical-align: middle; margin-right: 1px;"> 24x7 rapid response

### <img src="./icons/icon1.png" style="height: 20px; vertical-align: middle; margin-right: 1px;"> Adapting to weak light and strong light challenging scenes

### <img src="./icons/icon2.png" style="height: 20px; vertical-align: middle; margin-right: 1px;"> Effectively deal with optical attacks from physical hackers

### <img src="./icons/icon5.png" style="height: 20px; vertical-align: middle; margin-right: 1px;"> Adaptive dynamic multi-source sensor information fusion mechanism

<div align=center><img src="demo_image.png" width =100%></div>

<h2 align='center' style="text-align: center;"><img src="./icons/icon6.png" style="height: 25px; vertical-align: middle; margin-right: 1px;"> Introduction</h2>

1) Our project is developed based on the well-known SLAM framework [vins-fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion).

2) We use [ZeroDCE](https://github.com/Li-Chongyi/Zero-DCE)(Zero-Reference Deep Curve Estimation for Low-Light Image Enhancement) network to enhance the visual information of camera images, which improves the viability of the visual front-end in weak light and strong light challenging scenes

3) We introduces [SuperPoint feature](https://github.com/magicleap/SuperPointPretrainedNetwork) and rebuild front-end of vins-fusion for feature point detecting and tracking.

4) We also use [G2O](https://github.com/RainerKuemmerle/g2o) (General Graph Optimization) libiary to reconstruct the back-end of vins-fusion to speed up BA optimization.

5) We designed and implemented an Adaptive Adjustment Strategy For The Weight of Visual Inertial Information Fusion, which enable the system to effectively deal with optical injection attacks of physical hackers

<div align=center>
  <img src="demo_images/demo2.png" width =24%>
  <img src="demo_images/demo4.png" width =44%>
</div>

<h2 align='center' style="text-align: center;"><img src="./icons/icon7.png" style="height: 25px; vertical-align: middle; margin-right: 1px;"> Build Project and Run Demo</h2>

#### System requirements

```
ubuntu 20.04 and higher

ROS2 foxy and colcon build tools
```

#### Dependences

#### Build commands

#### Run

<h2 align='center' style="text-align: center;"><img src="./icons/icon8.png" style="height: 25px; vertical-align: middle; margin-right: 1px;"> About ROS2 Foxy</h2>

<h2 align='center' style="text-align: center;"><img src="./icons/icon9.png" style="height: 25px; vertical-align: middle; margin-right: 1px;"> Papers and Related Works</h2>

<h4 align='center' style="text-align: center;">Developed by the Security of Unmanned-System Laboratory<br>Northwestern Polytechnical University (NWPU, China)</h4>
