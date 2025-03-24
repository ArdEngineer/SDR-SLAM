#!/usr/bin/env python3.8
import argparse
import glob
import numpy as np
import os
import time

import cv2
import torch

# 警告：确保安装了OpenCV 3版本。
if int(cv2.__version__[0]) < 3:  # pragma: no cover
    print('警告：未安装OpenCV 3')

# 定义色彩映射。
myjet = np.array([[0.        , 0.        , 0.5       ],
                  [0.        , 0.        , 0.99910873],
                  [0.        , 0.37843137, 1.        ],
                  [0.        , 0.83333333, 1.        ],
                  [0.30044276, 1.        , 0.66729918],
                  [0.66729918, 1.        , 0.30044276],
                  [1.        , 0.90123457, 0.        ],
                  [1.        , 0.48002905, 0.        ],
                  [0.99910873, 0.07334786, 0.        ],
                  [0.5       , 0.        , 0.        ]])

# 定义 SuperPoint 网络结构。
class SuperPointNet(torch.nn.Module):
    """ Pytorch定义的SuperPoint网络。 """
    def __init__(self):
        super(SuperPointNet, self).__init__()
        self.relu = torch.nn.ReLU(inplace=True)
        self.pool = torch.nn.MaxPool2d(kernel_size=2, stride=2)
        c1, c2, c3, c4, c5, d1 = 64, 64, 128, 128, 256, 256
        # 共享编码器。
        self.conv1a = torch.nn.Conv2d(1, c1, kernel_size=3, stride=1, padding=1)
        self.conv1b = torch.nn.Conv2d(c1, c1, kernel_size=3, stride=1, padding=1)
        self.conv2a = torch.nn.Conv2d(c1, c2, kernel_size=3, stride=1, padding=1)
        self.conv2b = torch.nn.Conv2d(c2, c2, kernel_size=3, stride=1, padding=1)
        self.conv3a = torch.nn.Conv2d(c2, c3, kernel_size=3, stride=1, padding=1)
        self.conv3b = torch.nn.Conv2d(c3, c3, kernel_size=3, stride=1, padding=1)
        self.conv4a = torch.nn.Conv2d(c3, c4, kernel_size=3, stride=1, padding=1)
        self.conv4b = torch.nn.Conv2d(c4, c4, kernel_size=3, stride=1, padding=1)
        # 检测头。
        self.convPa = torch.nn.Conv2d(c4, c5, kernel_size=3, stride=1, padding=1)
        self.convPb = torch.nn.Conv2d(c5, 65, kernel_size=1, stride=1, padding=0)
        # 描述子头。
        self.convDa = torch.nn.Conv2d(c4, c5, kernel_size=3, stride=1, padding=1)
        self.convDb = torch.nn.Conv2d(c5, d1, kernel_size=1, stride=1, padding=0)

    def forward(self, x):
        """ 前向传播，联合计算未处理的点和描述子张量。
        输入
          x: 图像pytorch张量，形状N x 1 x H x W。
        输出
          semi: 输出点pytorch张量，形状N x 65 x H/8 x W/8。
          desc: 输出描述子pytorch张量，形状N x 256 x H/8 x W/8。
        """
        # 共享编码器。
        x = self.relu(self.conv1a(x))
        x = self.relu(self.conv1b(x))
        x = self.pool(x)
        x = self.relu(self.conv2a(x))
        x = self.relu(self.conv2b(x))
        x = self.pool(x)
        x = self.relu(self.conv3a(x))
        x = self.relu(self.conv3b(x))
        x = self.pool(x)
        x = self.relu(self.conv4a(x))
        x = self.relu(self.conv4b(x))
        # 检测头。
        cPa = self.relu(self.convPa(x))
        semi = self.convPb(cPa)
        # 描述子头。
        cDa = self.relu(self.convDa(x))
        desc = self.convDb(cDa)
        dn = torch.norm(desc, p=2, dim=1) # 计算范数。
        desc = desc.div(torch.unsqueeze(dn, 1)) # 除以范数进行归一化。
        return semi, desc

# 定义 SuperPoint 前端
class  SuperPointFrontend(object):
    """ 包装pytorch网络以帮助进行图像预处理和后处理。 """
    def __init__(self, weights_path, nms_dist, conf_thresh, nn_thresh,
                 cuda=False):
        self.name = 'SuperPoint'
        self.cuda = cuda
        self.nms_dist = nms_dist
        self.conf_thresh = conf_thresh
        self.nn_thresh = nn_thresh # L2描述子距离用于好的匹配。
        self.cell = 8 # 输出单元的大小。保持这个固定。
        self.weights_path=weights_path
        self.border_remove = 4 # 移除靠近边界的点。
        # 加载网络，进入推理模式。
        self.net = SuperPointNet()
        if cuda:
            # 在GPU上训练，在GPU上部署。
            self.net.load_state_dict(torch.load(weights_path))
            self.net = self.net.cuda()
        else:
            # 在GPU上训练，在CPU上部署。
            self.net.load_state_dict(torch.load(weights_path,
                                 map_location=lambda storage, loc: storage))
        self.net.eval()
        if cuda == True:
            print('[SP INFO] CUDA = ON')
            print('[SP INFO] CUDA AVAILABLE = ' + str(torch.cuda.is_available()))
        else:
            print('[SP INFO] CUDA = OFF')
        # print("ok")
        

    def nms_fast(self, in_corners, H, W, dist_thresh):
        """
        运行更快的近似非极大值抑制（NMS）在numpy corners上，形状为：
          3xN [x_i,y_i,conf_i]^T
        
        算法摘要：创建一个大小为HxW的网格。给每个角点位置分配1，其余为零。
        迭代所有的1，并将其转换为-1或0。
        通过将附近值设为0来抑制点。
        
        网格值说明：
        -1 : 保留。
         0 : 空或抑制。
         1 : 待处理（转换为保留或抑制）。
        
        注意：NMS首先将点四舍五入到整数，所以NMS距离可能不完全是dist_thresh。
        它还假设点在图像边界内。
        
        输入
          in_corners - 3xN numpy数组，角点 [x_i, y_i, confidence_i]^T。
          H - 图像高度。
          W - 图像宽度。
          dist_thresh - 抑制的距离，测量为无穷范数距离。
        返回
          nmsed_corners - 3xN numpy矩阵，经过NMS处理的角点。
          nmsed_inds - N长度numpy向量，经过NMS处理的角点索引。
        """
        grid = np.zeros((H, W)).astype(int) # 跟踪NMS数据。
        inds = np.zeros((H, W)).astype(int) # 存储点的索引。
                # 按置信度从高到低排序，并获取排序后的索引。
        inds1 = np.argsort(-in_corners[2,:])
        corners = in_corners[:,inds1]
        rcorners = corners[:2,:].round().astype(int) # 四舍五入角点到整数。

        # 如果没有角点，返回空数组。
        if rcorners.shape[1] == 0:
            return np.zeros((3,0)).astype(int), np.zeros(0).astype(int)
        if rcorners.shape[1] == 1:
            out = np.vstack((rcorners, in_corners[2])).reshape(3,1)
            return out, np.zeros((1)).astype(int)

        # 初始化网格，将所有角点位置设为1。
        for i, rc in enumerate(rcorners.T):
            grid[rcorners[1,i], rcorners[0,i]] = 1
            inds[rcorners[1,i], rcorners[0,i]] = i

        # 扩展网格边缘，以便处理靠近边缘的点。
        pad = dist_thresh
        grid = np.pad(grid, ((pad,pad), (pad,pad)), mode='constant')

        # 迭代所有的角点，执行非极大值抑制。
        count = 0
        for i, rc in enumerate(rcorners.T):
            pt = (rc[0]+pad, rc[1]+pad)
            if grid[pt[1], pt[0]] == 1: # 如果该点尚未被抑制。
                grid[pt[1]-pad:pt[1]+pad+1, pt[0]-pad:pt[0]+pad+1] = 0
                grid[pt[1], pt[0]] = -1
                count += 1

        # 获取所有存活的角点，并返回排序后的角点数组。
        keepy, keepx = np.where(grid==-1)
        keepy, keepx = keepy - pad, keepx - pad
        inds_keep = inds[keepy, keepx]
        out = corners[:, inds_keep]
        values = out[-1, :]
        inds2 = np.argsort(-values)
        out = out[:, inds2]
        out_inds = inds1[inds_keep[inds2]]
        return out, out_inds

    def run(self, img):
        # cv2.imshow("Python get" ,img)
        # cv2.waitKey(0)
        """处理numpy图像以提取点和描述子。
        输入
          img - HxW numpy float32输入图像，在范围[0,1]内。
        输出
          corners - 3xN numpy数组，角点 [x_i, y_i, confidence_i]^T。
          desc - 256xN numpy数组，对应的单位归一化描述子。
          heatmap - HxW numpy热图，在范围[0,1]内的点置信度。
        """
        
        assert img.dtype == np.float32, '图像必须是float32。'
        assert img.ndim == 2, '图像必须是灰度图像。'
        H, W = img.shape[0], img.shape[1]
        inp = img.copy()
        inp = (inp.reshape(1, H, W))
        inp = torch.from_numpy(inp)
        inp = torch.autograd.Variable(inp).view(1, 1, H, W)
        if self.cuda:
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
                inp = inp.cuda()
                print('[SP INFO] Load tensor to GPU successfully')
            else:
                print('[SP WARN] Runtime error : can not load tensor to GPU')
        # 网络前向传播。
        outs = self.net.forward(inp)
        semi, coarse_desc = outs[0], outs[1]
        # 将pytorch转换为numpy。
        semi = semi.data.cpu().numpy().squeeze()
        # --- 处理点。
        dense = np.exp(semi) # Softmax。
        dense = dense / (np.sum(dense, axis=0)+.00001) # 应该总和为1。
        # 移除垃圾。
        nodust = dense[:-1, :, :]
        # 调整形状以获得全分辨率热图。
        Hc = int(H / self.cell)
        Wc = int(W / self.cell)
        nodust = nodust.transpose(1, 2, 0)
        heatmap = np.reshape(nodust, [Hc, Wc, self.cell, self.cell])
        heatmap = np.transpose(heatmap, [0, 2, 1, 3])
        heatmap = np.reshape(heatmap, [Hc*self.cell, Wc*self.cell])
        xs, ys = np.where(heatmap >= self.conf_thresh) # 置信度阈值。
        if len(xs) == 0:
            return np.zeros((3, 0)), None, None
        pts = np.zeros((3, len(xs))) # 填充点数据大小为3xN。
        pts[0, :] = ys
        pts[1, :] = xs
        pts[2, :] = heatmap[xs, ys]
        pts, _ = self.nms_fast(pts, H, W, dist_thresh=self.nms_dist) # 应用NMS。
        inds = np.argsort(pts[2,:])
        pts = pts[:,inds[::-1]] # 按置信度排序。
        # 移除边界附近的点。
        bord = self.border_remove
        toremoveW = np.logical_or(pts[0, :] < bord, pts[0, :] >= (W-bord))
        toremoveH = np.logical_or(pts[1, :] < bord, pts[1, :] >= (H-bord))
        toremove = np.logical_or(toremoveW, toremoveH)
        pts = pts[:, ~toremove]
        # --- 处理描述子。
        D = coarse_desc.shape[1]
        if pts.shape[1] == 0:
            desc = np.zeros((D, 0))
        else:
            # 通过2D点位置插值。
            samp_pts = torch.from_numpy(pts[:2, :].copy())
            samp_pts[0, :] = (samp_pts[0, :] / (float(W)/2.)) - 1.
            samp_pts[1, :] = (samp_pts[1, :] / (float(H)/2.)) - 1.
            samp_pts = samp_pts.transpose(0, 1).contiguous()
            samp_pts = samp_pts.view(1, 1, -1, 2)
            samp_pts = samp_pts.float()
            if self.cuda:
                samp_pts = samp_pts.cuda()
            desc = torch.nn.functional.grid_sample(coarse_desc, samp_pts)
            desc = desc.data.cpu().numpy().reshape(D, -1)
            desc /= np.linalg.norm(desc, axis=0)[np.newaxis, :]
        # print("[PYTHON LOG] pts = ",pts, "\n")
        # print("[PYTHON LOG] desc",desc, "\n")
        return pts, desc, heatmap

