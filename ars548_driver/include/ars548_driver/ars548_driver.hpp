/**
 * @file ars548_driver.hpp
 * 
 * @brief ars548_driver is a class that is used to obtain all of the data from the sensor, translates it and sends it to the user for later use.
 * It also translates part of the received data into standard ROS topics (PointCloud...) so that the users can easily access to them 
 * For example PointClouds can be represented by Rviz
 */
#ifndef ARS548_DRIVER_HPP
#define ARS548_DRIVER_HPP

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <atomic>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "ars548_messages/msg/status.hpp"
#include "ars548_messages/msg/detection_list.hpp"
#include "ars548_messages/msg/object_list.hpp"
#include "ars548_data.h"

#define MSGBUFSIZE 102400

class ars548_driver : public rclcpp::Node{    
    public:
    /**
     * @brief  ars548_driver Node. Collects data from the ARS548 RDI sensor and sends the received data in various topics
     */
    ars548_driver();
    
    ~ars548_driver();

    private:
    bool override_stamp;
    std::string radar_ip;
    int ars548_port;
    std::string ars548_multicast_ip;
    std::string ars548_local_ip;
    std::string frame_ID;
    
    std::atomic<int> fd;
    char msgbuf[MSGBUFSIZE];
    
    // Threading
    std::unique_ptr<std::thread> receive_thread_;
    std::atomic<bool> run_thread_;

    // Publishers
    rclcpp::Publisher<ars548_messages::msg::Status>::SharedPtr statusPublisher;
    rclcpp::Publisher<ars548_messages::msg::ObjectList>::SharedPtr objectsPublisher;
    rclcpp::Publisher<ars548_messages::msg::DetectionList>::SharedPtr detectionsPublisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr directionPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr objectsCloudPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detectionsCloudPublisher;

    sensor_msgs::msg::PointCloud2 cloud_msgObj;
    sensor_msgs::msg::PointCloud2 cloud_msgDetect;
    geometry_msgs::msg::PoseArray cloud_Direction;
    
    sensor_msgs::PointCloud2Modifier modifierObject;
    sensor_msgs::PointCloud2Modifier modifierDetection;

    void receive_data_loop();
};

#endif // ARS548_DRIVER_HPP