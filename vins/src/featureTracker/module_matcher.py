#!/usr/bin/env python3.8

import argparse
import glob
import numpy as np
import os
import time

import cv2
import torch

# 定义点追踪器
class PointTracker(object):
    """管理固定内存中的特征点和描述子，实现稀疏光流点追踪。

    内部，追踪器存储一个 'tracks' 矩阵，大小为 M x (2+L)，M个追踪，最大长度为L，
    每一行对应：
    row_m = [track_id_m, avg_desc_score_m, point_id_0_m, ..., point_id_L-1_m]。
    """
    def __init__(self, max_length, nn_thresh):
        if max_length < 2:
            raise ValueError('max_length 必须大于或等于2。')
        self.maxl = max_length
        self.nn_thresh = nn_thresh
        self.all_pts = []
        for n in range(self.maxl):
            self.all_pts.append(np.zeros((2, 0)))
        self.last_desc = None
        self.tracks = np.zeros((0, self.maxl+2))
        self.track_count = 0
        self.max_score = 9999

    def nn_match_two_way(self, desc1, desc2, nn_thresh):
        """对两组描述子进行双向最近邻匹配，使得从描述子A到B的最近邻匹配必须等于从B到A的匹配。
        输入：
          desc1 - NxM numpy矩阵，N个相应的M维描述子。
          desc2 - NxM numpy矩阵，N个相应的M维描述子。
          nn_thresh - 可选的描述子距离，低于此值视为好的匹配。
        返回：
          matches - 3xL numpy数组，L个匹配，每列i是两个描述子的匹配，d_i在图像1中，d_j'在图像2中：
                    [d_i 索引, d_j' 索引, 匹配分数]^T
        """
        
        # print(desc1)
        desc1 = desc1.T
        desc2 = desc2.T
        # print('------------------------' + str(type(desc1) + str(type(desc2))))
        # print('OK')
        assert desc1.shape[0] == desc2.shape[0]
        # print('OK')
        if desc1.shape[1] == 0 or desc2.shape[1] == 0:
            return np.zeros((3, 0))
        if nn_thresh < 0.0:
            raise ValueError('\'nn_thresh\' 应该是非负数')
        # 计算L2距离。因为向量已经归一化，所以这很容易。
        dmat = np.dot(desc1.T, desc2)
        dmat = np.sqrt(2-2*np.clip(dmat, -1, 1))
        # 获取最近邻索引和分数。
        idx = np.argmin(dmat, axis=1)
        scores = dmat[np.arange(dmat.shape[0]), idx]
        # 根据阈值筛选匹配。
        keep = scores < nn_thresh
        # 检查最近邻是否双向匹配并保留这些匹配。
        idx2 = np.argmin(dmat, axis=0)
        keep_bi = np.arange(len(idx)) == idx2[idx]
        keep = np.logical_and(keep, keep_bi)
        idx = idx[keep]
        scores = scores[keep]
        # 获取存活的点索引。
        m_idx1 = np.arange(desc1.shape[1])[keep]
        m_idx2 = idx
        # 填充最终的3xN匹配数据结构。
        matches = np.zeros((3, int(keep.sum())))
        matches[0, :] = m_idx1
        matches[1, :] = m_idx2
        matches[2, :] = scores
        # print(matches)
        return matches

    def get_offsets(self):
        """迭代列表中的点并累积偏移值，用于将全局点ID索引到点列表中。
        返回
          offsets - N长度数组，存储整数偏移位置。
        """
        offsets = []
        offsets.append(0)
        for i in range(len(self.all_pts)-1):  # Skip last camera size, not needed.
            offsets.append(self.all_pts[i].shape[1])
        offsets = np.array(offsets)
        offsets = np.cumsum(offsets)
        return offsets

    def update(self, pts, desc):
        """向追踪器添加一组新的点和描述子。
        输入
          pts - 3xN numpy数组的2D点观测值。
          desc - DxN numpy数组的相应D维描述子。
        """
        if pts is None or desc is None:
            print('PointTracker: 警告，没有点被添加到追踪器。')
            return
        assert pts.shape[1] == desc.shape[1]
        # 初始化last_desc。
        if self.last_desc is None:
            self.last_desc = np.zeros((desc.shape[0], 0))
        # 移除最旧的点，存储其大小以更新ID。
        remove_size = self.all_pts[0].shape[1]
        self.all_pts.pop(0)
        self.all_pts.append(pts)
        # 移除追踪中最早的点。
        self.tracks = np.delete(self.tracks, 2, axis=1)
        # 更新追踪偏移。
        for i in range(2, self.tracks.shape[1]):
            self.tracks[:, i] -= remove_size
        self.tracks[:, 2:][self.tracks[:, 2:] < -1] = -1
        offsets = self.get_offsets()
        # 添加一个新的-1列。
        self.tracks = np.hstack((self.tracks, -1*np.ones((self.tracks.shape[0], 1))))
        # 尝试追加到现有追踪。
        matched = np.zeros((pts.shape[1])).astype(bool)
        matches = self.nn_match_two_way(self.last_desc, desc, self.nn_thresh)
        for match in matches.T:
            # 为匹配的追踪添加一个新点。
            id1 = int(match[0]) + offsets[-2]
            id2 = int(match[1]) + offsets[-1]
            found = np.argwhere(self.tracks[:, -2] == id1)
            if found.shape[0] > 0:
                matched[int(match[1])] = True
                row = int(found)
                self.tracks[row, -1] = id2
                if self.tracks[row, 1] == self.max_score:
                    # 初始化追踪分数。
                    self.tracks[row, 1] = match[2]
                else:
                    # 更新追踪分数，使用运行平均值。
                    # 注意(dd)：这个运行平均值可能包含来自旧匹配的分数
                    # 不包含在最后的max_length跟踪点中。
                    track_len = (self.tracks[row, 2:] != -1).sum() - 1.
                    frac = 1. / float(track_len)
                    self.tracks[row, 1] = (1.-frac)*self.tracks[row, 1] + frac*match[2]
        # 添加未匹配的追踪。
        new_ids = np.arange(pts.shape[1]) + offsets[-1]
        new_ids = new_ids[~matched]
        new_tracks = -1*np.ones((new_ids.shape[0], self.maxl + 2))
        new_tracks[:, -1] = new_ids
        new_num = new_ids.shape[0]
        new_trackids = self.track_count + np.arange(new_num)
        new_tracks[:, 0] = new_trackids
        new_tracks[:, 1] = self.max_score*np.ones(new_ids.shape[0])
        self.tracks = np.vstack((self.tracks, new_tracks))
        self.track_count += new_num # 更新追踪计数。
        # 移除空追踪。
        keep_rows = np.any(self.tracks[:, 2:] >= 0, axis=1)
        self.tracks = self.tracks[keep_rows, :]
        # 存储最后的描述子。
        self.last_desc = desc.copy()
        return

    def get_tracks(self, min_length):
        """检索给定最小长度的点追踪。
        输入
          min_length - 整数 >= 1，最小追踪长度
        输出
          returned_tracks - M x (2+L)大小的矩阵，存储追踪索引，其中
            M是追踪的数量，L是最大追踪长度。
        """
        if min_length < 1:
            raise ValueError('\'min_length\' 太小了。')
        valid = np.ones((self.tracks.shape[0])).astype(bool)
        good_len = np.sum(self.tracks[:, 2:] != -1, axis=1) >= min_length
        # 移除没有在最后一帧中观测到的追踪。
        not_headless = (self.tracks[:, -1] != -1)
        keepers = np.logical_and.reduce((valid, good_len, not_headless))
        returned_tracks = self.tracks[keepers, :].copy()
        return returned_tracks


