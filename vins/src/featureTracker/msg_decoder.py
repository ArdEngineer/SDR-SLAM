#!/usr/bin/env python3.8
import pickle

def msg_decoder(serialized_data):
    """
    反序列化接收到的二进制数据，返回特征点和描述子。
    :param serialized_data: 二进制字符串
    :return: keypoints, descriptors
    """
    features_dict = pickle.loads(serialized_data)
    keypoints = features_dict.get('keypoints', [])
    descriptors = features_dict.get('descriptors', [])
    return keypoints, descriptors