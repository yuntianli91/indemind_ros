/*
 * @Description: ROS常用头文件集合
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-11-01 14:16:25
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-11-01 14:34:36
 */
#ifndef COMMON_HEADER_H_
#define COMMON_HEADER_H_
// ================== ROS相关头文件 =================== //
#include <ros/ros.h> //核心库
#include <std_msgs/Header.h> //消息头
#include <sensor_msgs/Imu.h> //IMU消息
#include <sensor_msgs/Image.h> //图像消息
#include <sensor_msgs/image_encodings.h> //图像消息编码
#include <geometry_msgs/Pose.h> //位姿消息
#include <cv_bridge/cv_bridge.h> //ROS与OpenCV转换
// ================== C++标准库头文件 ================= //
#include <stdlib.h> //标准库
#include <memory> //智能指针库
#include <cmath> //数学运算库
#include <mutex> //智能锁库(多线程)
#include <thread> //线程库(多线程)
// ================== 第三方库头文件 ================== //
#include "indemind_driver/DriverInterface.h" //硬件API
#include <opencv2/opencv.hpp> //OpenCV

#endif