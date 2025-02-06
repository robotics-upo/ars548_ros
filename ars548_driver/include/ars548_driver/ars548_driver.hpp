/**
 * @file ars548_driver.hpp
 * 
 * @brief ars548_driver is a class that is used to obtain all of the data from the sensor, translates it and sends it to the user for later use.
 * It also translates part of the received data into standard ROS topics (PointCloud...) so that the users can easily access to them 
 * For example PointClouds can be represented by Rviz
 */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "ars548_messages/msg/status.hpp"
#include "ars548_messages/msg/detection_list.hpp"
#include "ars548_messages/msg/object_list.hpp"
#include "ars548_data.h"
using namespace std::chrono_literals;

/**
 * @brief Data obtained from the RadarSensors_Annex_AES548_IO SW 05.48.04.pdf 
 */
#define ARS548_DEFAULT_LOCAL_IP "10.13.1.166"
#define ARS548_DEFAULT_IP "10.13.1.113"
#define ARS548_MULTICAST_IP "224.0.2.2"
#define ARS548_DEFAULT_RADAR_PORT 42102
#define DEFAULT_FRAME_ID "ARS_548" 
#define MSGBUFSIZE 102400

class ars548_driver : public rclcpp::Node{    
    private:
    bool override_stamp = true;
    std::string radar_ip;
    int methodID;
    char msgbuf[MSGBUFSIZE];
    int fd;
    int nbytes;
    float AbsVel;
    std::string answer;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
    
    sensor_msgs::msg::PointCloud2 cloud_msgObj;
    sensor_msgs::msg::PointCloud2 cloud_msgDetect;
    geometry_msgs::msg::PoseArray cloud_Direction;

    sensor_msgs::PointCloud2Modifier modifierObject;
    sensor_msgs::PointCloud2Modifier modifierDetection;
    /**
     * @brief  Sends the data on socket fd to the address addr.
     * 
     * @tparam data The buffer with all of the data.
     * @param fd The socket where you are going to send the data.
     * @param addr The address where the data is going to be sent.
     * @return nbytes. The number of bytes sent. If its value is -1 there has been an error.
     */
    template<typename T>
    int SendMessage(int fd, T& data, sockaddr_in addr){
        int nbytes=sendto(fd,&data,8+sizeof(data),0,(struct sockaddr *) &addr,sizeof(addr));
        return nbytes;
    }
    
    private:
    
    
    /**
     * @brief Reads the data received from the radar and sends it to the user and Rviz2.
     * 
     * @return The status of the connection. If it returns 1, there is an error in the execution.
     */
    int readData() {
       
        //These are the publishers that send the data in a custom message
        auto statusPublisher = create_publisher<ars548_messages::msg::Status>("Status",10);
        auto objectsPublisher = create_publisher<ars548_messages::msg::ObjectList>("ObjectList",10);
        auto detectionsPublisher = create_publisher<ars548_messages::msg::DetectionList>("DetectionList",10);
        //These are the publishers that send the data to Rviz2
        auto directionPublisher = create_publisher<geometry_msgs::msg::PoseArray>("DirectionVelocity",10);
        auto objectsCloudPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudObject",10);
        auto detectionsCloudPublisher =create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudDetection",10);
        //Create messages for the publishers
        auto statusMessage=ars548_messages::msg::Status();
        auto detectionMessage=ars548_messages::msg::DetectionList();
        auto objectMessage=ars548_messages::msg::ObjectList(); 
        
        fd= socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0) {
            perror("socket");
            return 1;
        }
        struct sockaddr_in addr;
        u_int yes = 1;
        if (
            setsockopt(
                fd, SOL_SOCKET, SO_REUSEADDR, (char*) &yes, sizeof(yes)
            ) < 0
        ){
            perror("Reusing ADDR failed");
            return 1;
        }

        // set up destination address
        //
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY); // differs from sender
        addr.sin_port = htons(ars548_Port);
        // bind to receive address
        //
        if (bind(fd, (struct sockaddr*) &addr, sizeof(addr)) < 0) {
            perror("bind");
            return 1;
        }

        // use setsockopt() to request that the kernel join a multicast group
        //
        struct ip_mreq mreq;
        mreq.imr_multiaddr.s_addr = inet_addr(ars548_multicast_ip.c_str());
        mreq.imr_interface.s_addr = inet_addr(ars548_local_ip.c_str());
      
        if (
            setsockopt(
                fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq, sizeof(mreq)
            ) < 0
        ){
            perror("setsockopt");
            return 1;
        }
        unsigned int addrlen = sizeof(addr);
        // now just enter a read-print loop
        //
        while (1)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"getting data.");
            nbytes = recvfrom(
            fd,
            msgbuf,
            MSGBUFSIZE,
            0,
            (struct sockaddr *) &addr,
            &addrlen
            );
            if(nbytes < 0){
                perror("Failed attempt of getting data");
                return 1;
            }
            switch (nbytes)
            {
            case STATUS_MESSAGE_PAYLOAD:

                struct UDPStatus status;
                if(status.receiveStatusMsg(nbytes,msgbuf))
                {
                    statusMessage = status.toMsg();
                    statusPublisher->publish(statusMessage);
                    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"publishing status data.\n");
                }
                break;
            case OBJECT_MESSAGE_PAYLOAD:
                struct ObjectList object_list;
                object_list=*((struct ObjectList *)msgbuf);
                object_list.changeEndianness();
                if(object_list.isValid()) {
                    objectsPublisher->publish(object_list.toMsg(frame_ID, override_stamp));
                    object_list.fillObjectCloud(cloud_msgObj, modifierObject, frame_ID, override_stamp);
                    objectsCloudPublisher->publish(cloud_msgObj);
                    directionPublisher->publish(object_list.getDirectionMessage(frame_ID, override_stamp));
                }
                break;
            case DETECTION_MESSAGE_PAYLOAD:
                struct DetectionList detection_list;
                detection_list = *((struct DetectionList *)msgbuf);
                detection_list.changeEndianness();
                if (detection_list.isValid()) {
                    detectionsPublisher->publish(detection_list.toMsg(frame_ID, override_stamp));
                    detection_list.fillDetectionCloud(cloud_msgDetect, modifierDetection, frame_ID, override_stamp);
                    detectionsCloudPublisher->publish(cloud_msgDetect);
                }
                break;
            }
        }
    }

    public:
    std::string ars548_ip, ars548_multicast_ip, ars548_local_ip;
    std::string frame_ID;
    static int ars548_Port;
    
    /**
     * @brief  ars548_driver Node. Collects data from the ARS548 RDI sensor and sends the received data in various topics
     */
    ars548_driver():Node("ars_548_driver"),modifierObject(cloud_msgObj),modifierDetection(cloud_msgDetect){
        //Parameter declaration so the user can change them
        this->declare_parameter("localIP", ARS548_DEFAULT_LOCAL_IP);
        this->declare_parameter("radarIP", ARS548_DEFAULT_IP);
        this->declare_parameter("radarPort", ARS548_DEFAULT_RADAR_PORT);
        this->declare_parameter("frameID", DEFAULT_FRAME_ID);
        this->declare_parameter("multicastIP", ARS548_MULTICAST_IP);
        this->declare_parameter("overrideStamp", true);
        rclcpp::Clock clock;
        this->ars548_local_ip = this->get_parameter("localIP").as_string();
        this->ars548_ip=this->get_parameter("radarIP").as_string();
        this->ars548_multicast_ip=this->get_parameter("multicastIP").as_string();
        this->ars548_Port=this->get_parameter("radarPort").as_int();
        this->frame_ID=this->get_parameter("frameID").as_string();
        this->override_stamp = this->get_parameter("overrideStamp").as_bool();
        //Creation of their modifiers
        
        //Set fields and size of every PointCloud
        //Object Cloud
         modifierObject.setPointCloud2Fields(5,
            "x",1,sensor_msgs::msg::PointField::FLOAT32,
            "y",1,sensor_msgs::msg::PointField::FLOAT32,
            "z",1,sensor_msgs::msg::PointField::FLOAT32,
            "vx",1,sensor_msgs::msg::PointField::FLOAT32,
            "vy",1,sensor_msgs::msg::PointField::FLOAT32
        );
        modifierObject.clear();
        modifierObject.reserve(ARS548_MAX_OBJECTS);
        
        //Detection Cloud
        modifierDetection.setPointCloud2Fields(8,
            "x",1,sensor_msgs::msg::PointField::FLOAT32,
            "y",1,sensor_msgs::msg::PointField::FLOAT32,
            "z",1,sensor_msgs::msg::PointField::FLOAT32,
            "v",1,sensor_msgs::msg::PointField::FLOAT32,
            "r",1,sensor_msgs::msg::PointField::FLOAT32,
            "RCS",1,sensor_msgs::msg::PointField::INT8,
            "azimuth",1,sensor_msgs::msg::PointField::FLOAT32,
            "elevation",1,sensor_msgs::msg::PointField::FLOAT32
        );
        modifierDetection.clear();
        modifierDetection.reserve(ARS548_MAX_DETECTIONS);
        cloud_Direction.poses.reserve(ARS548_MAX_OBJECTS);
        //handler subscription to the three callbacks so we can see if they have been changed
        readData();
    }
};
//std::string ars548_driver::ars548_ = ARS548_DEFAULT_IP;
//std::string ars548_driver::ars548_ARS548_MULTICAST_IP = ARS548_MULTICAST_IP;
int ars548_driver::ars548_Port = ARS548_DEFAULT_RADAR_PORT;