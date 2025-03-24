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

#include "feature_tracker.h"

#include <iostream>
#include <stdio.h>

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*--------------------------------------------------DEFINITION OF SUPERPOINT MODULES--------------------------------------------------*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PyObject *pmodule;
PyObject *pointtracker;
PyObject *argofpointertracker;
PyObject *tracker;
PyObject* py_module_pickle;
PyObject* PyModule_Decoder;
PyObject* Decoder;

#include <vector>
#include <string.h>
using namespace std;

#define DEBUG_INFO cout << "[DEBUG INFO] OK" << endl;

PyObject *cached_tracker_nn_match_two_way = nullptr;
PyObject *cached_decoder = nullptr;

// 全局变量用于存储接收到的特征信息
// std::vector<float> features_msg_;
std::vector<float> keypoints_msg_;
std::vector<float> descriptors_msg_;
bool keypoints_received_ = false;
bool descriptors_received_ = false;
std::mutex mutex_;
std::condition_variable cond_;

void CacheMethods()
{
    // 缓存 tracker.nn_match_two_way 方法
    cached_tracker_nn_match_two_way = PyObject_GetAttrString(tracker, "nn_match_two_way");
    if (!cached_tracker_nn_match_two_way) {
        PyErr_Print();
        throw std::runtime_error("[CPython API ERROR] Failed to cache method 'tracker.nn_match_two_way'.");
    }

    // 缓存 msg_decoder 方法
    cached_decoder = PyObject_GetAttrString(PyModule_Decoder, "msg_decoder");
    if (!cached_decoder) {
        PyErr_Print();
        throw std::runtime_error("[CPython API ERROR] Failed to cache method 'msg_decoder'.");
    }
}

rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
// rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr feature_subscriber;
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr keypoints_subscriber;
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr descriptors_subscriber;

// void featureData_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
// {
//     std::lock_guard<std::mutex> lock(mutex_);
//     features_msg_ = msg->data;
//     features_received_ = true;
//     cond_.notify_one();
// }

void keypoints_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    keypoints_msg_ = msg->data;
    keypoints_received_ = true;
    cond_.notify_one();
}

void descriptors_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    descriptors_msg_ = msg->data;
    descriptors_received_ = true;
    cond_.notify_one();
}

void InitSuperpoint(std::shared_ptr<rclcpp::Node> n)
{
    // Py_SetPythonHome(L"/usr");
    Py_Initialize();
    _import_array();//直接使用import_array会报错，import_array其实是一个宏定义，里面调用了_import_array

    int max_length=5;
    int min_length=2;

    //添加文件路径
    PyRun_SimpleString("print('[PYTHON INFO] python init successfully')");
    PyRun_SimpleString("import sys");

    PyRun_SimpleString("sys.path.append('/home/slam/Desktop/DemoSLAM/src/vins/src/featureTracker/')");
    PyRun_SimpleString("print('[PYTHON INFO] route append successfully')");

    //调用demo
    pmodule=PyImport_ImportModule("module_matcher");
    pointtracker=PyObject_GetAttrString(pmodule,"PointTracker");
    //初始化tracker的参数
    argofpointertracker=Py_BuildValue("(id)",max_length,0.5);

    py_module_pickle = PyImport_ImportModule("pickle");
    if (!py_module_pickle) {
        PyErr_Print();  // 打印错误信息
        std::cerr << "[SP WARN] Failed to import pickle module." << std::endl;
        return;
    }

    PyModule_Decoder = PyImport_ImportModule("msg_decoder");
    if (!PyModule_Decoder) {
        PyErr_Print();
        std::cerr << "[SP WARN] Failed to import msg_decoder module." << std::endl;
        return;
    }
    Decoder = PyObject_GetAttrString(PyModule_Decoder, "msg_decoder");
    if (!Decoder) {
        PyErr_Print();
        std::cerr << "[SP WARN] Failed to get 'msg_decoder' function." << std::endl;
        return;
    }

    if(!pmodule)
    {
        cout<<"[SP WARN] Import python failed\n";
    }

    tracker=PyEval_CallObject(pointtracker,argofpointertracker);

    if(tracker) cout<<"[SP INFO] Tracker init successfully\n";
    else cout<<"[SP WARN] Tracker init failed\n";

    // 缓存方法
    CacheMethods();

    // 创建图像发布者
    image_publisher = n->create_publisher<sensor_msgs::msg::Image>
    ("image_from_featrueTracker", 10);

    // 创建特征点订阅者
    keypoints_subscriber = n->create_subscription<std_msgs::msg::Float32MultiArray>(
        "keypoints_topic", 10, keypoints_callback);

    descriptors_subscriber = n->create_subscription<std_msgs::msg::Float32MultiArray>(
        "descriptors_topic", 10, descriptors_callback);
}

void FinishSuperpoint()
{
    // 清理缓存的方法
    if (cached_tracker_nn_match_two_way) {
        Py_DECREF(cached_tracker_nn_match_two_way);
        cached_tracker_nn_match_two_way = nullptr;
    }

    Py_Finalize();
    cout<<"[SP INFO] Python finished successfully"<<endl;
}

void MagicPoint_Extractor(const cv::Mat & img,FRAME_PTS_INFO &ptsinfo){
    keypoints_msg_.clear();
    descriptors_msg_.clear();

    // 将图像转换为 ROS 2 消息并发布
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img).toImageMsg();
    image_publisher->publish(*img_msg);

    std::unique_lock<std::mutex> lock(mutex_);
    while (!(keypoints_received_ && descriptors_received_) && rclcpp::ok())
    {
        cond_.wait(lock);
    }

    if (keypoints_received_ && descriptors_received_)
    {
        // 提取 keypoints
        size_t num_keypoints = keypoints_msg_.size() / 2;
        std::vector<cv::Point2f> keypoints;
        for (size_t i = 0; i < num_keypoints; ++i)
        {
            float x = keypoints_msg_[i * 2];
            float y = keypoints_msg_[i * 2 + 1];
            keypoints.push_back(cv::Point2f(x, y));
        }

        // 提取 descriptors
        // 提取 descriptors
        size_t descriptor_length = 256;  // 每个描述子的长度
        size_t num_descriptors = descriptors_msg_.size() / descriptor_length;
        std::vector<float> descriptors(descriptors_msg_);
        // 将描述子封装为 numpy.ndarray 类型的 PyObject
        float* descriptor0data = new float[num_descriptors * descriptor_length];
        for (int i =0;i<descriptors.size();i++)
        {
            descriptor0data[i]=descriptors[i];
        }

        npy_intp dims[] = {num_descriptors, descriptor_length};
        PyObject* py_descriptors = PyArray_SimpleNewFromData(2, dims, NPY_FLOAT, (void*)descriptor0data);
        cout << py_descriptors->ob_type->tp_name << endl;


        // 存储到 frameinfo
        ptsinfo.pts = keypoints;
        ptsinfo.decs = py_descriptors;
    }

    // 重置标志
    keypoints_received_ = false;
    descriptors_received_ = false;
}

vector <cv::Point3f> SuperMatcher(PyObject *desc1,PyObject *desc2)
{
    TicToc match_time;
    vector <cv::Point3f> ret;
    // PyObject *matches = PyObject_CallMethod(tracker,"nn_match_two_way","OOd",desc1,desc2,0.7);
    PyObject *matches = PyObject_CallObject(cached_tracker_nn_match_two_way, PyTuple_Pack(3, desc1, desc2, PyFloat_FromDouble(0.5)));
    if(matches==NULL){
        cout << "Error\n";
    }
    // 将PyObject转换为PyArrayObject
    PyArrayObject *array = reinterpret_cast<PyArrayObject*>(matches);

    // 确认数组是double类型且形状为(3, L)
    if (PyArray_TYPE(array) != NPY_DOUBLE || PyArray_NDIM(array) != 2 || PyArray_DIM(array, 0) != 3) {
        PyErr_SetString(PyExc_TypeError, "Expected a (3, L) numpy array of doubles.");
        PyErr_Print();
        Py_DECREF(matches);
        return ret;
    }

    npy_intp cols = PyArray_DIM(array, 1);
    double* data = static_cast<double*>(PyArray_DATA(array));

    // 遍历每一列，构建cv::Point3f并添加到结果向量中
    for (npy_intp i = 0; i < cols; ++i) {
        cv::Point3f point;
        point.x = data[i];           // 第一行的第i个元素
        point.y = data[i + cols];    // 第二行的第i个元素
        point.z = data[i + 2 * cols]; // 第三行的第i个元素
        ret.push_back(point);
    }
    std::cout << "[SP INFO] Match costs : " << match_time.toc() << " ms." << std::endl;

    Py_DECREF(matches);

    return ret;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*------------------------------------------------END DEFINITION OF SUPERPOINT MODULES------------------------------------------------*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*------------------------------------------------DEFINITION OF TRACKER SUPPORT MODULES-----------------------------------------------*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

FeatureTracker::FeatureTracker(){
    n_id = 0;
}

void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        printf("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);
    }
    return;
}

void FeatureTracker::drawTrack(const cv::Mat &curImg, const cv::Mat &prevImg, 
                               vector<cv::Point2f> curPts, 
                               const vector<cv::Point2f> & prevPts)
{
    //int rows = imLeft.rows;
    int cols =prevImg.cols;
    imTrack = prevImg.clone();
    cv::hconcat(prevImg, curImg, imTrack);
    cv::cvtColor(imTrack, imTrack, cv::COLOR_GRAY2RGB);

    for (size_t j = 0; j < prevPts.size(); j++)
    {
        cv::circle(imTrack, prevPts[j], 2, cv::Scalar(int(prevPts.size()) - j, 0, 255), 2);
    }
    for (size_t j = 0; j < curPts.size(); j++)
    {
        curPts[j].x += cols;
        cv::circle(imTrack, curPts[j], 2, cv::Scalar(int(curPts.size()) - j, 0, 255), 2);
    }
    
    for(size_t i = 0; i < matches.size(); i++){
        if(matches[i].z < 0.5){
            cv::line(imTrack, prevPts[matches[i].x], curPts[matches[i].y], cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }

    return;
}

cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}

vector<cv::Point2f> FeatureTracker::normalizedPlaneMapping(vector<cv::Point2f> &pts, camodocal::CameraPtr cam){
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}

map<int, cv::Point2f> FeatureTracker::createLandMarkIDMapping(vector<int> & ids, vector<cv::Point2f> & pts){
    map<int, cv::Point2f> ret_map;

    for(size_t i = 0; i < pts.size(); i++)
        ret_map[ids[i]] = pts[i];

    return ret_map;
}

void FeatureTracker::updateIDs(vector<int> & cur_ids, const vector<int> prev_ids){
    if(prev_ids.size() <= 0){
        for(size_t i = 0; i < cur_frame.feature.pts.size(); i++)
            cur_ids.push_back(n_id++);
    }else{
        cur_ids.resize(cur_frame.feature.pts.size());
        std::fill(cur_ids.begin(), cur_ids.end(), -1);
        for(size_t i = 0; i < matches.size(); i++){
            if(matches[i].z < 0.5)
                cur_ids[matches[i].y] = prev_ids[matches[i].x];
        }
        for(size_t i = 0; i < cur_ids.size(); i++){
            if(cur_ids[i] == -1)
                cur_ids[i] = n_id++;
        }
    }
}

vector<cv::Point2f> FeatureTracker::ptsVelocityCalc(vector<int> &ids, vector<cv::Point2f> &pts, map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> velocity;

    if(prev_id_pts.size() > 0){
        double dt = cur_frame.time - prev_frame.time;
        for(size_t i = 0; i < pts.size(); i++){
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                velocity.push_back(cv::Point2f(0, 0));
        }
    }else{
        for(size_t i = 0; i < pts.size(); i++){
            velocity.push_back(cv::Point2f(0, 0));
        }
    }

    return velocity;
}

void FeatureTracker::clearFrameInfo(FRAME_INFO & p){
    p.ids.clear();
    p.feature.pts.clear();
    p.feature.decs = NULL;
    p.un_pts.clear();
    p.pts_velocity.clear();
    p.landmark_pts_map.clear();

    return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*----------------------------------------------END DEFINITION OF TRACKER SUPPORT MODULES---------------------------------------------*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*------------------------------------------------DEFINITION OF MAIN FUNC : TRACK_IMAGE-----------------------------------------------*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1){
    cv::Mat right_img = _img1; // NULL

    // Get new image from message
    clearFrameInfo(cur_frame);
    cur_frame.time = _cur_time;
    cur_frame.image = _img;

    // Extract magic points
    TicToc t_of_MagicPoint_API;
    MagicPoint_Extractor(cur_frame.image, cur_frame.feature);
    std::cout << "[SP INFO] Point extractor api cost : " << t_of_MagicPoint_API.toc() << " ms." << std::endl;

    // cout << "[CHECK] pts = " << endl;
    // for(auto pt : cur_frame.feature.pts){
    //     cout << "pts = " << pt.x << "," << pt.y << endl;
    // }

    // Match with pts of prev_frame
    if(prev_frame.feature.pts.size() > 0){
        TicToc t_of_Matcher_API;
        matches.clear();
        matches = SuperMatcher(prev_frame.feature.decs, cur_frame.feature.decs);
        std::cout << "[SP INFO] Point matcher api cost : " << t_of_Matcher_API.toc() << " ms." << std::endl;
    }

    // Normalized plane mapping
    cur_frame.un_pts = normalizedPlaneMapping(cur_frame.feature.pts, m_camera[0]);

    // Update global ID of new extracted points
    updateIDs(cur_frame.ids, prev_frame.ids);

    // Create landmark mapping to frame map
    cur_frame.landmark_pts_map = createLandMarkIDMapping(cur_frame.ids, cur_frame.un_pts);

    // Calculate pts_velocity of current frame 
    cur_frame.pts_velocity = ptsVelocityCalc(cur_frame.ids, cur_frame.un_pts, cur_frame.landmark_pts_map, prev_frame.landmark_pts_map);

    // if prev_img is not empty then show the track image
    if(int(prev_frame.feature.pts.size()) > 0){
        drawTrack(cur_frame.image, prev_frame.image, cur_frame.feature.pts, prev_frame.feature.pts);
    }

    // Make feature frame and return it
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (size_t i = 0; i < cur_frame.ids.size(); i++)
    {
        int feature_id = cur_frame.ids[i];

        double x, y ,z;
        x = cur_frame.un_pts[i].x;
        y = cur_frame.un_pts[i].y;
        z = 1;

        double p_u, p_v;
        p_u = cur_frame.feature.pts[i].x;
        p_v = cur_frame.feature.pts[i].y;

        int camera_id = 0;

        double velocity_x, velocity_y;
        velocity_x = cur_frame.pts_velocity[i].x;
        velocity_y = cur_frame.pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }

    // Update prev_frame and prepare for next frame
    clearFrameInfo(prev_frame);
    prev_frame = cur_frame;

    return featureFrame;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*--------------------------------------------END DEFINITION OF MAIN FUNC : TRACK_IMAGE-----------------------------------------------*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
