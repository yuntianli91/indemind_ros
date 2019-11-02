/*
 * @Description: Indemind双目惯性模组数据读取与发布节点
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-11-01 14:28:33
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-11-02 19:38:08
 */
#include "indemind_ros/common_headers.h"
using namespace indem;
// =============================== 全局变量 ===================================//
ros::Time imu_start_time; //IMU消息起始时间
ros::Time camera_start_time; //图像消息起始时间

int imu_id = 0; //IMU消息编号
int image_l_id = 0; //左相机图像标号
int image_r_id = 0; //右相机图像编号
// 消息Publisher
ros::Publisher imu_pub;
ros::Publisher image_l_pub;
ros::Publisher image_r_pub;
ros::Publisher camera_info_pub;
// 读取模组参数相关
int version = 255;
size_t paramSize = 0;
unsigned char *module_info = new unsigned char[FLASH_MAX_SIZE];
ModuleParamInFlash<1> moduleParam = {0};
// =================================== 全局函数 ============================== //
/**
 * @brief IMU的回调函数, 只进行IMU消息的生成和发布. 根据官方文档,请勿于此函数中进行负责
 *      操作,否则可能引起数据丢失.该函数的调用频率与设定的IMU数据采集频率一致.
 * @param data : IMU数据结构体;
 */
void ImuCallback(IMUData* data){
    sensor_msgs::Imu imu_msg;
    // 如果是第一条消息,则初始化时戳
    // 硬件的timeStamp是以ms为单位的, ros::Duration是以s为单位,需要转换
    if(imu_id == 0){
        imu_start_time = ros::Time::now() - ros::Duration(data->_timeStamp / 1000.);
    }
    // 构建消息头
    imu_msg.header.stamp = imu_start_time + ros::Duration(data->_timeStamp / 1000.);
    // ROS_INFO("IMU_Stamp: %lf", data->_timeStamp);
    imu_msg.header.seq = imu_id++;
    // 构建量测消息
    imu_msg.linear_acceleration.x = data->_acc[0];
    imu_msg.linear_acceleration.y = data->_acc[1];
    imu_msg.linear_acceleration.z = data->_acc[2];

    imu_msg.angular_velocity.x = data->_gyr[0];
    imu_msg.angular_velocity.y = data->_gyr[1];
    imu_msg.angular_velocity.z = data->_gyr[2];
    // 发布消息
    imu_pub.publish(imu_msg);
    return;
}
/**
 * @brief IMU的回调函数, 只进行IMU消息的生成和发布. 根据官方文档,请勿于此函数中进行负责
 *      操作,否则可能引起数据丢失.该函数的调用频率与设定的IMU数据采集频率一致.
 * @param data 
 */
void CameraCallback(cameraData* data){
    if(image_l_id == 0){
        camera_start_time = ros::Time::now() - ros::Duration(data->_timeStamp / 1000.);
    }
    // -------------------- 获取双目相机图像 -------------------------- //
    int img_height = data->_height;
    int img_width = data->_width;
    cv::Mat img_stereo(img_height, img_width, CV_8UC1, data->_image);
    // --------------------构建左相机图像消息 -------------------------- //
    cv::Mat img_l(img_stereo, cv::Rect(0, 0, img_width / 2, img_height));
    cv_bridge::CvImage cvi_l;
    sensor_msgs::Image img_msg_l;
    // 构建消息头
    cvi_l.header.stamp = camera_start_time + ros::Duration(data->_timeStamp / 1000.);
    cvi_l.header.seq = image_l_id++;
    cvi_l.encoding = "mono8";
    cvi_l.image = img_l;
    // 将cv::Mat转换为ros图像消息
    cvi_l.toImageMsg(img_msg_l);
    // 发布图像消息
    image_l_pub.publish(img_msg_l);
    // ------------------ 构建右相机图像消息 -------------------------- //
    cv::Mat img_r(img_stereo, cv::Rect(img_width / 2 - 1, 0, img_width / 2, img_height));
    cv_bridge::CvImage cvi_r;
    sensor_msgs::Image img_msg_r;
    // 构建消息头
    cvi_r.header.stamp = camera_start_time + ros::Duration(data->_timeStamp / 1000.);
    cvi_r.header.seq = image_r_id++;
    cvi_r.encoding = "mono8";
    cvi_r.image = img_r;
    // 将cv::Mat转换为ros图像消息
    cvi_r.toImageMsg(img_msg_r);
    // 发布图像消息
    image_r_pub.publish(img_msg_r);
    // ------------------ 释放资源 --------------------------------- //
    img_stereo.release();
    img_l.release();
    img_r.release();   
}
/**
 * @brief 将获取的模组各项参数上传ROS参数服务器
 * 
 */
void uploadParams(){

}
// ================================ 节点主函数 ================================ //

int main(int argc, char** argv){
    ros::init(argc, argv, "indemind_vi"); // 初始化ros节点
    ros::NodeHandle nh; // 初始化节点句柄
    // ---------------------- 初始化Publisher ------------------------------ //
    imu_pub = nh.advertise<sensor_msgs::Imu>("/indemind_vi/imu", 100);
    image_l_pub = nh.advertise<sensor_msgs::Image>("/indemind_vi/image_left", 10);
    image_r_pub = nh.advertise<sensor_msgs::Image>("/indemind_vi/image_right", 10);
    // ------------------------ 初始化API --------------------------------- //
    IDriverInterface *driver = DriverFactory();
    // ------------------- 从ROS参数服务器获取节点参数 ---------------------- //
    int img_width = 1280;
    int img_height = 800;
    int fps = 30;
    int imuFreq = 400;
    // -------------- 获取相机模组参数并上传ROS参数服务器 -------------------- //
   if(!driver->GetModuleParams(version, module_info, paramSize)){
        ROS_ERROR("Get device parameters failed !");
        memcpy(&moduleParam, module_info, paramSize);
    }
    // ------------------ 启动设备并设置传感器回调函数 ---------------------- //
    if(!driver->Open(imuFreq, fps)){
        ROS_ERROR("Open device failed !");
    }
    else{
        driver->SetIMUCallback(ImuCallback);
        driver->SetCameraCallback(CameraCallback);
    }

    while(ros::ok()){
        ros::spinOnce();
    }

    driver->Close();
    return 0;
}