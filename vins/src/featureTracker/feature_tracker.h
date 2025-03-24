/*******************************************************************************************
 * Copyright (C) 2024, UAV Navigation Security Llaboratory, School of Cyber-Security of NWPU
 * 
 * This file is part of SuperMono.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: ChenQY (ChineseUserChen@163.com.com)
 *******************************************************************************************/
#ifndef FEATURE_TRACKER_H
#define FEATURE_TRACKER_H
#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <sstream>
#include <mutex>
#include <condition_variable>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

#include <python3.8/Python.h>
#include <numpy/arrayobject.h>

#include <vector>
using namespace std;

extern PyObject *pmodule;
extern PyObject *pointtracker;
extern PyObject *argofpointertracker;
extern PyObject *tracker;
extern PyObject* py_module_pickle;
extern PyObject* PyModule_Decoder;
extern PyObject* Decoder;

struct FRAME_PTS_INFO{
    vector<cv::Point2f> pts;
    PyObject *decs;
};

struct FRAME_INFO{
    double time;
    cv::Mat image;
    FRAME_PTS_INFO feature;

    vector<int> ids;
    vector<cv::Point2f> un_pts;
    vector<cv::Point2f> pts_velocity;

    map<int, cv::Point2f> landmark_pts_map;
};

class FeatureTracker{
    public:
        FeatureTracker();
        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());

        void readIntrinsicParameter(const vector<string> &calib_file);

        vector<cv::Point2f> normalizedPlaneMapping(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
        vector<cv::Point2f> ptsVelocityCalc(vector<int> &ids, vector<cv::Point2f> &pts, map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
        void drawTrack(const cv::Mat &, const cv::Mat &, vector<cv::Point2f> , const vector<cv::Point2f> &);
        map<int, cv::Point2f> createLandMarkIDMapping(vector<int> & ids, vector<cv::Point2f> & pts);
        void updateIDs(vector<int> & cur_ids, const vector<int> prev_ids);
        void clearFrameInfo(FRAME_INFO &);

        cv::Mat getTrackImage();
        cv::Mat imTrack;
        
        FRAME_INFO cur_frame, prev_frame;

        map<int, cv::Point2f> prevLeftPtsMap;
        vector<camodocal::CameraPtr> m_camera;

        vector<cv::Point3f> matches;
        vector<int> track_cnt;

        int n_id;//一个全局变量，用于记录下一个可用的全局 ID。它从 0 开始，并在每次分配新特征点时递增
};

void InitSuperpoint(std::shared_ptr<rclcpp::Node>);
void FinishSuperpoint();
void MagicPoint_Extractor(const cv::Mat &, FRAME_PTS_INFO &);
vector<cv::Point3f> SuperMatcher(PyObject *, PyObject *);

void feature_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg);

extern rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
extern rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr keypoints_subscriber;
extern rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr descriptors_subscriber;

#endif FEATURE_TRACKER_H