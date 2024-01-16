#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "ars548_messages/msg/status.hpp"
#include "ars548_messages/msg/detection_list.hpp"
#include "ars548_messages/msg/object_list.hpp"
#include "ars548_data.h"
using namespace std::chrono_literals;

#define DEFAULT_RADAR_IP "224.0.2.2"
#define RADAR_INTERFACE "10.13.1.166"
#define DEFAULT_RADAR_PORT 42102
#define DEFAULT_FRAME_ID "ARS_548" 
#define MSGBUFSIZE 102400
#define MAX_OBJECTS 50
#define MAX_DETECTIONS 800
#define STATUS_MESSAGE_METHOD_ID 380
#define OBJECT_MESSAGE_METHOD_ID 329
#define DETECTION_MESSAGE_METHOD_ID 336
#define STATUS_MESSAGE_PDU_LENGTH 76
#define OBJECT_MESSAGE_PDU_LENGTH 9393
#define DETECTION_MESSAGE_PDU_LENGTH 35328
#define STATUS_MESSAGE_PAYLOAD 84
#define OBJECT_MESSAGE_PAYLOAD 9401
#define DETECTION_MESSAGE_PAYLOAD 35336
#define POINTCLOUD_HEIGHT 1
#define POINTCLOUD_WIDTH 1000
#define SIZE 1000

class ars548_driver : public rclcpp::Node{    
    
    private:
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
    
    template<typename T>
    int SendMessage(int fd, T& data, sockaddr_in addr){
    int nbytes=sendto(fd,&data,8+sizeof(data),0,(struct sockaddr *) &addr,sizeof(addr));
        return nbytes;
    }

    template<typename T>
    T ToLittleEndian(T v){
        T r;
        uint8_t *pv = (uint8_t *)&v, *pr = (uint8_t *)&r;
        for (int i =0;i <int(sizeof(T)); i++){
            pr[i]=pv[sizeof(T)-1-i];
        }
        return r;
    }

    template<typename T>
    T ToBigEndian(T data){
        T r;

        uint8_t *pdata=(uint8_t *)&data, *pr=(uint8_t *)&r;
        for (int i = 0; i < int(sizeof(T)); i++)
        {
            pr[i]=pdata[sizeof(T)-1-i];
        }
        return r;
        
    }
    //This function reads the data the radar is Sending to the computer, after that, it sends a fragment of the data to rviz2 for visualization.
    // It also sends all the data in a few custom messages for personal use. 
    int readData(rclcpp::Clock clock){
        auto node=rclcpp::Node::make_shared("publisherObj");
        //These are the publishers that send the data in a custom message
        auto statusPublisher=node->create_publisher<ars548_messages::msg::Status>("StatusCloud",10);
        auto objectPublisher=node->create_publisher<ars548_messages::msg::ObjectList>("ObjectCloud",10);
        auto detectionsPublisher=node->create_publisher<ars548_messages::msg::DetectionList>("DetectionCloud",10);
        //These are the publishers that send the data to Rviz2
        auto directionPublisher=node->create_publisher<geometry_msgs::msg::PoseArray>("DirectionVelocity",10);
        auto pubObj= node->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudObject",10);
        auto pubDetect=node->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudDetection",10);
        //Create messages for the publishers
        auto statusMessage=ars548_messages::msg::Status();
        auto detectionMessage=ars548_messages::msg::DetectionList();
        auto objectMessage=ars548_messages::msg::ObjectList(); 
        cloud_Direction.poses.resize(100);
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
        mreq.imr_multiaddr.s_addr = inet_addr(this->ars548_IP.c_str());
        mreq.imr_interface.s_addr = inet_addr(RADAR_INTERFACE);
       
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
            
            nbytes = recvfrom(
            fd,
            msgbuf,
            MSGBUFSIZE,
            0,
            (struct sockaddr *) &addr,
            &addrlen
            );
            //Creation of the Point Cloud iterators.
            //Detection Iterators
            sensor_msgs::PointCloud2Iterator<float> iter_xD(cloud_msgDetect,"x");
            sensor_msgs::PointCloud2Iterator<float> iter_yD(cloud_msgDetect,"y");
            sensor_msgs::PointCloud2Iterator<float> iter_zD(cloud_msgDetect,"z");
            //Object Iterators
            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msgObj,"x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msgObj,"y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msgObj,"z");
            if(nbytes<0){
                perror("Failed attempt of getting data");
                return 1;
            }
            switch (nbytes)
            {
            case STATUS_MESSAGE_PAYLOAD:
                struct UDPStatus status;
                status = *((struct UDPStatus *)msgbuf);
                status.ServiceID=ToLittleEndian(status.ServiceID);
                status.MethodID=ToLittleEndian(status.MethodID);
                status.PayloadLength=ToLittleEndian(status.PayloadLength);
                std::cout<< status.ServiceID<<", "<<status.MethodID<<", "<<status.PayloadLength<<std::endl;
                if(status.MethodID==STATUS_MESSAGE_METHOD_ID && status.PayloadLength==STATUS_MESSAGE_PDU_LENGTH){
                    status.Timestamp_Nanoseconds=ToLittleEndian(status.Timestamp_Nanoseconds);
                    status.Timestamp_Seconds=ToLittleEndian(status.Timestamp_Seconds);
                    status.Longitudinal=ToLittleEndian(status.Longitudinal);
                    status.Lateral=ToLittleEndian(status.Lateral);
                    status.Vertical=ToLittleEndian(status.Vertical);
                    status.Yaw=ToLittleEndian(status.Yaw);
                    status.Pitch=ToLittleEndian(status.Pitch);
                    status.Length=ToLittleEndian(status.Length);
                    status.Width=ToLittleEndian(status.Width);
                    status.Height=ToLittleEndian(status.Height);
                    status.Wheelbase=ToLittleEndian(status.Wheelbase);
                    status.MaximunDistance=ToLittleEndian(status.MaximunDistance);
                    status.SensorIPAddress_0=ToLittleEndian(status.SensorIPAddress_0);
                    status.SensorIPAddress_1=ToLittleEndian(status.SensorIPAddress_1);
                    //std::cout<<"SyncStatus: "<<status.Timestamp_SyncStatus<<std::endl;
                    statusMessage.cycletime=status.CycleTime;
                    statusMessage.configurationcounter=status.ConfigurationCounter;
                    statusMessage.frequencyslot=status.FrequencySlot;
                    statusMessage.hcc=status.HCC;
                    statusMessage.height=status.Height;
                    statusMessage.lateral=status.Lateral;
                    statusMessage.length=status.Length;
                    statusMessage.longitudinal=status.Longitudinal;
                    statusMessage.maximundistance=status.MaximunDistance;
                    statusMessage.pitch=status.Pitch;
                    statusMessage.plugorientation=status.PlugOrientation;
                    statusMessage.powersave_standstill=status.PayloadLength;
                    statusMessage.sensoripaddress_0=status.SensorIPAddress_0;
                    statusMessage.sensoripaddress_1=status.SensorIPAddress_1;
                    statusMessage.status_blockagestatus=status.Status_BlockageStatus;
                    statusMessage.status_characteristicspeed=status.Status_CharacteristicSpeed;
                    statusMessage.status_drivingdirection=status.Status_DrivingDirection;
                    statusMessage.status_lateralacceleration=status.Status_LateralAcceleration;
                    statusMessage.status_longitudinalacceleration=status.Status_LongitudinalAcceleration;
                    statusMessage.status_longitudinalvelocity=status.Status_LongitudinalVelocity;
                    statusMessage.status_radarstatus=status.Status_RadarStatus;
                    statusMessage.status_steeringangle=status.Status_SteeringAngle;
                    statusMessage.status_temperaturestatus=status.Status_TemperatureStatus;
                    statusMessage.status_voltagestatus=status.Status_VoltageStatus;
                    statusMessage.status_yawrate=status.Status_YawRate;
                    statusMessage.swversion_major=status.SWVersion_Major;
                    statusMessage.swversion_minor=status.SWVersion_Minor;
                    statusMessage.swversion_patch=status.SWVersion_Patch;
                    statusMessage.timeslot=status.TimeSlot;
                    statusMessage.timestamp_nanoseconds=status.Timestamp_Nanoseconds;
                    statusMessage.timestamp_seconds=status.Timestamp_Seconds;
                    statusMessage.timestamp_syncstatus=status.Timestamp_SyncStatus;
                    statusMessage.vertical=status.Vertical;
                    statusMessage.wheelbase=status.Wheelbase;
                    statusMessage.width=status.Width;
                    statusMessage.yaw=status.Yaw;
                    statusPublisher->publish(statusMessage);
                }
                break;
            case OBJECT_MESSAGE_PAYLOAD:
                struct Object_List object_List;
                object_List=*((struct Object_List *)msgbuf);
                object_List.ServiceID=ToLittleEndian(object_List.ServiceID);
                object_List.MethodID=ToLittleEndian(object_List.MethodID);
                object_List.PayloadLength=ToLittleEndian(object_List.PayloadLength);
                //Setting the ars548_messages
                std::cout<< object_List.ServiceID<<", "<<object_List.MethodID<<", "<<object_List.PayloadLength<<std::endl;
                if(object_List.MethodID==OBJECT_MESSAGE_METHOD_ID && object_List.PayloadLength==OBJECT_MESSAGE_PDU_LENGTH){
                     //Changes all of the > 8bit data to little endian
                        object_List.CRC=ToLittleEndian(object_List.CRC);
                        object_List.Length=ToLittleEndian(object_List.Length);
                        object_List.SQC=ToLittleEndian(object_List.SQC);
                        object_List.DataID=ToLittleEndian(object_List.DataID);
                        object_List.Timestamp_Nanoseconds=ToLittleEndian(object_List.Timestamp_Nanoseconds);
                        object_List.Timestamp_Seconds=ToLittleEndian(object_List.Timestamp_Seconds);
                        object_List.EventDataQualifier=ToLittleEndian(object_List.EventDataQualifier);
                        objectMessage.crc=object_List.CRC;
                        objectMessage.length=object_List.Length;
                        objectMessage.sqc=object_List.SQC;
                        objectMessage.timestamp_nanoseconds=object_List.Timestamp_Nanoseconds;
                        objectMessage.timestamp_seconds=object_List.Timestamp_Seconds;
                        objectMessage.eventdataqualifier=object_List.EventDataQualifier;
                        objectMessage.extendedqualifier=object_List.ExtendedQualifier;
                        objectMessage.dataid=object_List.DataID;
                        objectMessage.objectlist_numofobjects=object_List.ObjectList_NumOfObjects;
                        objectMessage.timestamp_syncstatus=object_List.Timestamp_SyncStatus;
                        
                        objectMessage.header.frame_id=this->frame_ID;
                        objectMessage.header.stamp=clock.now();
                        std::cout<<"Number of objects:"<<object_List.ObjectList_NumOfObjects<<std::endl;
                        if (object_List.ObjectList_NumOfObjects>50){
                            object_List.ObjectList_NumOfObjects=50;
                        }
                        //Changes all of the > 8bit data to little endian inside the 50 element array
                        for(u_int32_t i =0; i<object_List.ObjectList_NumOfObjects;++i,++iter_x,++iter_y,++iter_z/*,++iterVel*/){
                            object_List.ObjectList_Objects[i].u_StatusSensor=ToLittleEndian(object_List.ObjectList_Objects[i].u_StatusSensor);
                            object_List.ObjectList_Objects[i].u_ID=ToLittleEndian(object_List.ObjectList_Objects[i].u_ID);
                            object_List.ObjectList_Objects[i].u_Age=ToLittleEndian(object_List.ObjectList_Objects[i].u_Age);
                            object_List.ObjectList_Objects[i].u_Position_InvalidFlags=ToLittleEndian(object_List.ObjectList_Objects[i].u_Position_InvalidFlags);
                            object_List.ObjectList_Objects[i].u_Position_X=ToLittleEndian(object_List.ObjectList_Objects[i].u_Position_X);
                            object_List.ObjectList_Objects[i].u_Position_X_STD=ToLittleEndian(object_List.ObjectList_Objects[i].u_Position_X_STD);
                            object_List.ObjectList_Objects[i].u_Position_Y=ToLittleEndian(object_List.ObjectList_Objects[i].u_Position_Y);
                            object_List.ObjectList_Objects[i].u_Position_Y_STD=ToLittleEndian(object_List.ObjectList_Objects[i].u_Position_Y_STD);
                            object_List.ObjectList_Objects[i].u_Position_Z=ToLittleEndian(object_List.ObjectList_Objects[i].u_Position_Z);
                            object_List.ObjectList_Objects[i].u_Position_Z_STD=ToLittleEndian(object_List.ObjectList_Objects[i].u_Position_Z_STD);
                            object_List.ObjectList_Objects[i].u_Position_CovarianceXY=ToLittleEndian(object_List.ObjectList_Objects[i].u_Position_CovarianceXY);
                            object_List.ObjectList_Objects[i].u_Position_Orientation=ToLittleEndian(object_List.ObjectList_Objects[i].u_Position_Orientation);
                            object_List.ObjectList_Objects[i].u_Position_Orientation_STD=ToLittleEndian(object_List.ObjectList_Objects[i].u_Position_Orientation_STD);
                            object_List.ObjectList_Objects[i].u_Existence_Probability=ToLittleEndian(object_List.ObjectList_Objects[i].u_Existence_Probability);
                            object_List.ObjectList_Objects[i].u_Existence_PPV=ToLittleEndian(object_List.ObjectList_Objects[i].u_Existence_PPV);
                            object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X);
                            object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X_STD=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X_STD);
                            object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y);
                            object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y_STD=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y_STD);
                            object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_CovarianceXY=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_CovarianceXY);
                            object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X);
                            object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X_STD=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X_STD);
                            object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y);
                            object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y_STD=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y_STD);
                            object_List.ObjectList_Objects[i].f_Dynamics_RelVel_CovarianceXY=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_CovarianceXY);
                            object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X);
                            object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X_STD=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X_STD);
                            object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y);
                            object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y_STD=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y_STD);
                            object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_CovarianceXY=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_CovarianceXY);
                            object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_X=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_Y_STD);
                            object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_CovarianceXY=ToLittleEndian(object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_CovarianceXY);
                            object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_Mean=ToLittleEndian(object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_Mean);
                            object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_STD=ToLittleEndian(object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_STD);
                            object_List.ObjectList_Objects[i].u_Shape_Length_Status=ToLittleEndian(object_List.ObjectList_Objects[i].u_Shape_Length_Status);
                            object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean=ToLittleEndian(object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean);
                            object_List.ObjectList_Objects[i].u_Shape_Length_Edge_STD=ToLittleEndian(object_List.ObjectList_Objects[i].u_Shape_Length_Edge_STD);
                            object_List.ObjectList_Objects[i].u_Shape_Width_Status=ToLittleEndian(object_List.ObjectList_Objects[i].u_Shape_Width_Status);
                            object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean=ToLittleEndian(object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean);
                            object_List.ObjectList_Objects[i].u_Shape_Width_Edge_STD=ToLittleEndian(object_List.ObjectList_Objects[i].u_Shape_Width_Edge_STD);
                            
                            objectMessage.objectlist_objects[i].u_statussensor=object_List.ObjectList_Objects[i].u_StatusSensor;
                            objectMessage.objectlist_objects[i].u_id=object_List.ObjectList_Objects[i].u_ID;
                            objectMessage.objectlist_objects[i].u_age=object_List.ObjectList_Objects[i].u_Age;
                            objectMessage.objectlist_objects[i].u_position_invalidflags=object_List.ObjectList_Objects[i].u_Position_InvalidFlags;
                            objectMessage.objectlist_objects[i].u_position_x=object_List.ObjectList_Objects[i].u_Position_X;
                            objectMessage.objectlist_objects[i].u_position_x_std=object_List.ObjectList_Objects[i].u_Position_X_STD;
                            objectMessage.objectlist_objects[i].u_position_y=object_List.ObjectList_Objects[i].u_Position_Y;
                            objectMessage.objectlist_objects[i].u_position_y_std=object_List.ObjectList_Objects[i].u_Position_Y_STD;
                            objectMessage.objectlist_objects[i].u_position_z=object_List.ObjectList_Objects[i].u_Position_Z;
                            objectMessage.objectlist_objects[i].u_position_z_std=object_List.ObjectList_Objects[i].u_Position_Z_STD;
                            objectMessage.objectlist_objects[i].u_position_covariancexy=object_List.ObjectList_Objects[i].u_Position_CovarianceXY;
                            objectMessage.objectlist_objects[i].u_position_orientation=object_List.ObjectList_Objects[i].u_Position_Orientation;
                            objectMessage.objectlist_objects[i].u_position_orientation_std=object_List.ObjectList_Objects[i].u_Position_Orientation_STD;
                            objectMessage.objectlist_objects[i].u_existence_invalidflags=object_List.ObjectList_Objects[i].u_Existence_InvalidFlags;
                            objectMessage.objectlist_objects[i].u_existence_ppv=object_List.ObjectList_Objects[i].u_Existence_PPV;
                            objectMessage.objectlist_objects[i].u_existence_probability=object_List.ObjectList_Objects[i].u_Existence_Probability;
                            objectMessage.objectlist_objects[i].u_dynamics_absaccel_invalidflags=object_List.ObjectList_Objects[i].u_Dynamics_AbsAccel_InvalidFlags;
                            objectMessage.objectlist_objects[i].u_dynamics_absvel_invalidflags=object_List.ObjectList_Objects[i].u_Dynamics_AbsVel_InvalidFlags;
                            objectMessage.objectlist_objects[i].u_dynamics_orientation_invalidflags=object_List.ObjectList_Objects[i].u_Dynamics_Orientation_InvalidFlags;
                            objectMessage.objectlist_objects[i].u_dynamics_orientation_rate_mean=object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_Mean;
                            objectMessage.objectlist_objects[i].u_dynamics_orientation_rate_std=object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_STD;
                            objectMessage.objectlist_objects[i].u_dynamics_relaccel_invalidflags=object_List.ObjectList_Objects[i].u_Dynamics_RelAccel_InvalidFlags;
                            objectMessage.objectlist_objects[i].u_dynamics_relvel_invalidflags=object_List.ObjectList_Objects[i].u_Dynamics_RelVel_InvalidFlags;
                            objectMessage.objectlist_objects[i].u_shape_length_edge_invalidflags=object_List.ObjectList_Objects[i].u_Shape_Length_Edge_InvalidFlags;
                            objectMessage.objectlist_objects[i].u_shape_length_edge_mean=object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean;
                            objectMessage.objectlist_objects[i].u_shape_length_edge_std=object_List.ObjectList_Objects[i].u_Shape_Length_Edge_STD;
                            objectMessage.objectlist_objects[i].u_shape_length_status=object_List.ObjectList_Objects[i].u_Shape_Length_Status;
                            objectMessage.objectlist_objects[i].u_shape_width_edge_invalidflags=object_List.ObjectList_Objects[i].u_Shape_Width_Edge_InvalidFlags;
                            objectMessage.objectlist_objects[i].u_shape_width_edge_mean=object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean;
                            objectMessage.objectlist_objects[i].u_shape_width_edge_std=object_List.ObjectList_Objects[i].u_Shape_Length_Edge_STD;
                            objectMessage.objectlist_objects[i].u_shape_width_status=object_List.ObjectList_Objects[i].u_Shape_Width_Status;
                            objectMessage.objectlist_objects[i].u_statusmeasurement=object_List.ObjectList_Objects[i].u_StatusMeasurement;
                            objectMessage.objectlist_objects[i].u_statusmovement=object_List.ObjectList_Objects[i].u_StatusMovement;
                            objectMessage.objectlist_objects[i].u_statussensor=object_List.ObjectList_Objects[i].u_StatusSensor;
                            objectMessage.objectlist_objects[i].u_classification_animal=object_List.ObjectList_Objects[i].u_Classification_Animal;
                            objectMessage.objectlist_objects[i].u_classification_bicycle=object_List.ObjectList_Objects[i].u_Classification_Bicycle;
                            objectMessage.objectlist_objects[i].u_classification_car=object_List.ObjectList_Objects[i].u_Classification_Car;
                            objectMessage.objectlist_objects[i].u_classification_hazard=object_List.ObjectList_Objects[i].u_Classification_Hazard;
                            objectMessage.objectlist_objects[i].u_classification_motorcycle=object_List.ObjectList_Objects[i].u_Classification_Motorcycle;
                            objectMessage.objectlist_objects[i].u_classification_overdrivable=object_List.ObjectList_Objects[i].u_Classification_Overdrivable;
                            objectMessage.objectlist_objects[i].u_classification_pedestrian=object_List.ObjectList_Objects[i].u_Classification_Pedestrian;
                            objectMessage.objectlist_objects[i].u_classification_truck=object_List.ObjectList_Objects[i].u_Classification_Truck;
                            objectMessage.objectlist_objects[i].u_classification_underdrivable=object_List.ObjectList_Objects[i].u_Classification_Underdrivable;
                            objectMessage.objectlist_objects[i].u_classification_unknown=object_List.ObjectList_Objects[i].u_Position_CovarianceXY;
                            objectMessage.objectlist_objects[i].f_dynamics_absaccel_covariancexy=object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_CovarianceXY;
                            objectMessage.objectlist_objects[i].f_dynamics_absaccel_x=object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X;
                            objectMessage.objectlist_objects[i].f_dynamics_absaccel_x_std=object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X_STD;
                            objectMessage.objectlist_objects[i].f_dynamics_absaccel_y=object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y;
                            objectMessage.objectlist_objects[i].f_dynamics_absaccel_y_std=object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y_STD;
                            objectMessage.objectlist_objects[i].f_dynamics_absvel_covariancexy=object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_CovarianceXY;
                            objectMessage.objectlist_objects[i].f_dynamics_absvel_x=object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X;
                            objectMessage.objectlist_objects[i].f_dynamics_absvel_x_std=object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X_STD;
                            objectMessage.objectlist_objects[i].f_dynamics_absvel_y=object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y;
                            objectMessage.objectlist_objects[i].f_dynamics_absvel_y_std=object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y_STD;
                            objectMessage.objectlist_objects[i].f_dynamics_relaccel_covariancexy=object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_CovarianceXY;
                            objectMessage.objectlist_objects[i].f_dynamics_relaccel_x=object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_X;
                            objectMessage.objectlist_objects[i].f_dynamics_relaccel_x_std=object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_X_STD;
                            objectMessage.objectlist_objects[i].f_dynamics_relaccel_y=object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_Y;
                            objectMessage.objectlist_objects[i].f_dynamics_relaccel_y_std=object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_Y_STD;
                            objectMessage.objectlist_objects[i].f_dynamics_relvel_covariancexy=object_List.ObjectList_Objects[i].f_Dynamics_RelVel_CovarianceXY;
                            objectMessage.objectlist_objects[i].f_dynamics_relvel_x=object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X;
                            objectMessage.objectlist_objects[i].f_dynamics_relvel_x_std=object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X_STD;
                            objectMessage.objectlist_objects[i].f_dynamics_relvel_y=object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y;
                            objectMessage.objectlist_objects[i].f_dynamics_relvel_y_std=object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y_STD;  
                            AbsVel=3.6*sqrt(pow(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X,2)+pow(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y,2));
                            //if (AbsVel>10){
                                cloud_msgObj.header=std_msgs::msg::Header();
                                cloud_msgObj.header.frame_id=this->frame_ID;
                                cloud_msgObj.header.stamp.nanosec=std::chrono::nanoseconds().count();
                                cloud_msgObj.header.stamp.sec=std::chrono::seconds().count();
                                cloud_msgObj.is_dense=true;
                                cloud_msgObj.is_bigendian=false;
                                cloud_msgObj.height=POINTCLOUD_HEIGHT;
                                cloud_msgObj.width=POINTCLOUD_WIDTH;
                                *iter_x=object_List.ObjectList_Objects[i].u_Position_X;
                                *iter_y=object_List.ObjectList_Objects[i].u_Position_Y;
                                *iter_z=object_List.ObjectList_Objects[i].u_Position_Z;
                                //To show the direction of the moving object
                            
                                cloud_Direction.header=std_msgs::msg::Header();
                                cloud_Direction.header.frame_id=this->frame_ID;
                                cloud_Direction.header.stamp.nanosec=std::chrono::nanoseconds().count();
                                cloud_Direction.header.stamp.sec=std::chrono::seconds().count();
                                cloud_Direction.poses[i].position.x=double(object_List.ObjectList_Objects[i].u_Position_X);
                                cloud_Direction.poses[i].position.y=double(object_List.ObjectList_Objects[i].u_Position_Y);
                                cloud_Direction.poses[i].position.z=double(object_List.ObjectList_Objects[i].u_Position_Z);
                                cloud_Direction.poses[i].orientation.x=double(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X);
                                cloud_Direction.poses[i].orientation.y=double(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y);
                                cloud_Direction.poses[i].orientation.z=0;
                                cloud_Direction.poses[i].orientation.w=0;
                                std::cout<<"Speed of object "<<i<<": "<< AbsVel<<"km/h"<<std::endl;
                            //}     
                        } 
                        pubObj->publish(cloud_msgObj);
                        directionPublisher->publish(cloud_Direction);
                        objectPublisher->publish(objectMessage);   
                }
                break;
            case DETECTION_MESSAGE_PAYLOAD:
                float posX, posY,posZ;
                cloud_msgDetect.height=POINTCLOUD_HEIGHT;
                cloud_msgDetect.width=POINTCLOUD_WIDTH;
                cloud_msgDetect.header=std_msgs::msg::Header();
                cloud_msgDetect.header.frame_id=this->frame_ID;
                cloud_msgDetect.header.stamp.nanosec=std::chrono::nanoseconds().count();
                cloud_msgDetect.header.stamp.sec=std::chrono::seconds().count();
                cloud_msgDetect.is_dense=true;
                cloud_msgDetect.is_bigendian=false;
                struct DetectionList detectionList;
                detectionList=*((struct DetectionList *)msgbuf);
                detectionList.ServiceID=ToLittleEndian(detectionList.ServiceID);
                detectionList.MethodID=ToLittleEndian(detectionList.MethodID);
                detectionList.PayloadLength=ToLittleEndian(detectionList.PayloadLength);
                std::cout<<detectionList.ServiceID<<", "<<detectionList.MethodID<<", "<<detectionList.PayloadLength<<std::endl;
                if(detectionList.MethodID==DETECTION_MESSAGE_METHOD_ID && detectionList.PayloadLength==DETECTION_MESSAGE_PDU_LENGTH){
                    detectionList.CRC=ToLittleEndian(detectionList.CRC);
                        detectionList.Length=ToLittleEndian(detectionList.Length);
                        detectionList.SQC=ToLittleEndian(detectionList.SQC);
                        detectionList.DataID=ToLittleEndian(detectionList.DataID);
                        detectionList.Timestamp_Nanoseconds=ToLittleEndian(detectionList.Timestamp_Nanoseconds);
                        detectionList.Timestamp_Seconds=ToLittleEndian(detectionList.Timestamp_Seconds);
                        detectionList.EventDataQualifier=ToLittleEndian(detectionList.EventDataQualifier);
                        detectionList.Origin_InvalidFlags=ToLittleEndian(detectionList.Origin_InvalidFlags);
                        detectionList.Origin_Xpos=ToLittleEndian(detectionList.Origin_Xpos);
                        detectionList.Origin_Xstd=ToLittleEndian(detectionList.Origin_Xstd);
                        detectionList.Origin_Ypos=ToLittleEndian(detectionList.Origin_Ypos);
                        detectionList.Origin_Ystd=ToLittleEndian(detectionList.Origin_Ystd);
                        detectionList.Origin_Zpos=ToLittleEndian(detectionList.Origin_Zpos);
                        detectionList.Origin_Zstd=ToLittleEndian(detectionList.Origin_Zstd);
                        detectionList.Origin_Roll=ToLittleEndian(detectionList.Origin_Roll);
                        detectionList.Origin_Rollstd=ToLittleEndian(detectionList.Origin_Rollstd);
                        detectionList.Origin_Pitch=ToLittleEndian(detectionList.Origin_Pitch);
                        detectionList.Origin_Pitchstd=ToLittleEndian(detectionList.Origin_Pitchstd);
                        detectionList.Origin_Yaw=ToLittleEndian(detectionList.Origin_Yaw);
                        detectionList.Origin_Yawstd=ToLittleEndian(detectionList.Origin_Yawstd);
                    
                        detectionMessage.header.frame_id=this->frame_ID;
                        detectionMessage.header.stamp=clock.now();
                        detectionMessage.aln_status=detectionList.Aln_Status;
                        detectionMessage.crc=detectionList.CRC;
                        detectionMessage.dataid=detectionList.DataID;
                        detectionMessage.eventdataqualifier=detectionList.EventDataQualifier;
                        detectionMessage.extendedqualifier=detectionList.ExtendedQualifier;
                        detectionMessage.length=detectionList.Length;
                        detectionMessage.origin_invalidflags=detectionList.Origin_InvalidFlags;
                        detectionMessage.origin_pitch=detectionList.Origin_Pitch;
                        detectionMessage.origin_pitchstd=detectionList.Origin_Pitchstd;
                        detectionMessage.origin_roll=detectionList.Origin_Roll;
                        detectionMessage.origin_rollstd=detectionList.Origin_Rollstd;
                        detectionMessage.origin_xpos=detectionList.Origin_Xpos;
                        detectionMessage.origin_xstd=detectionList.Origin_Xstd;
                        detectionMessage.origin_yaw=detectionList.Origin_Yaw;
                        detectionMessage.origin_yawstd=detectionList.Origin_Yawstd;
                        detectionMessage.origin_ypos=detectionList.Origin_Ypos;
                        detectionMessage.origin_ystd=detectionList.Origin_Ystd;
                        detectionMessage.origin_zpos=detectionList.Origin_Zpos;
                        detectionMessage.origin_zstd=detectionList.Origin_Zstd;
                        detectionMessage.sqc=detectionList.SQC;
                        detectionMessage.timestamp_nanoseconds=detectionList.Timestamp_Nanoseconds;
                        detectionMessage.timestamp_seconds=detectionList.Timestamp_Seconds;
                        detectionMessage.timestamp_syncstatus=detectionList.Timestamp_SyncStatus;
                        detectionList.List_NumOfDetections=ToLittleEndian(detectionList.List_NumOfDetections);
                        detectionList.List_RadVelDomain_Min=ToLittleEndian(detectionList.List_RadVelDomain_Min);
                        detectionList.List_RadVelDomain_Max=ToLittleEndian(detectionList.List_RadVelDomain_Max);
                        detectionList.Aln_AzimuthCorrection=ToLittleEndian(detectionList.Aln_AzimuthCorrection);
                        detectionList.Aln_ElevationCorrection=ToLittleEndian(detectionList.Aln_ElevationCorrection);
                        //Changes all of the > 8bit data to little endian inside the 800 elements array
                        for(uint64_t i=0; i<detectionList.List_NumOfDetections;i++,++iter_xD,++iter_yD,++iter_zD){
                            //Setting the detection data to littleEndian
                            detectionList.List_Detections[i].f_AzimuthAngle=ToLittleEndian(detectionList.List_Detections[i].f_AzimuthAngle);
                            detectionList.List_Detections[i].f_AzimuthAngleSTD=ToLittleEndian(detectionList.List_Detections[i].f_AzimuthAngleSTD);
                            detectionList.List_Detections[i].f_ElevationAngle=ToLittleEndian(detectionList.List_Detections[i].f_ElevationAngle);
                            detectionList.List_Detections[i].f_ElevationAngleSTD=ToLittleEndian(detectionList.List_Detections[i].f_ElevationAngleSTD);
                            detectionList.List_Detections[i].f_Range=ToLittleEndian(detectionList.List_Detections[i].f_Range);
                            detectionList.List_Detections[i].f_RangeSTD=ToLittleEndian(detectionList.List_Detections[i].f_RangeSTD);
                            detectionList.List_Detections[i].f_RangeRate=ToLittleEndian(detectionList.List_Detections[i].f_RangeRate);
                            detectionList.List_Detections[i].f_RangeRateSTD=ToLittleEndian(detectionList.List_Detections[i].f_RangeRateSTD);
                            detectionList.List_Detections[i].u_MeasurementID=ToLittleEndian(detectionList.List_Detections[i].u_MeasurementID);
                            detectionList.List_Detections[i].u_ObjectID=ToLittleEndian(detectionList.List_Detections[i].u_ObjectID);
                            detectionList.List_Detections[i].u_SortIndex=ToLittleEndian(detectionList.List_Detections[i].u_SortIndex);
                            //
                            detectionMessage.list_detections[i].f_azimuthangle=detectionList.List_Detections[i].f_AzimuthAngle;
                            detectionMessage.list_detections[i].f_azimuthanglestd=detectionList.List_Detections[i].f_AzimuthAngleSTD;
                            detectionMessage.list_detections[i].f_elevationangle=detectionList.List_Detections[i].f_ElevationAngle;
                            detectionMessage.list_detections[i].f_elevationanglestd=detectionList.List_Detections[i].f_ElevationAngleSTD;
                            detectionMessage.list_detections[i].f_range=detectionList.List_Detections[i].f_Range;
                            detectionMessage.list_detections[i].f_rangerate=detectionList.List_Detections[i].f_RangeRate;
                            detectionMessage.list_detections[i].f_rangeratestd=detectionList.List_Detections[i].f_RangeRateSTD;
                            detectionMessage.list_detections[i].f_rangestd=detectionList.List_Detections[i].f_RangeSTD;
                            detectionMessage.list_detections[i].s_rcs=detectionList.List_Detections[i].s_RCS;
                            detectionMessage.list_detections[i].u_ambiguityflag=detectionList.List_Detections[i].u_AmbiguityFlag;
                            detectionMessage.list_detections[i].u_classification=detectionList.List_Detections[i].u_Classification;
                            detectionMessage.list_detections[i].u_invalidflags=detectionList.List_Detections[i].u_InvalidFlags;
                            detectionMessage.list_detections[i].u_measurementid=detectionList.List_Detections[i].u_MeasurementID;
                            detectionMessage.list_detections[i].u_multitargetprobabilitym=detectionList.List_Detections[i].u_MultiTargetProbabilityM;
                            detectionMessage.list_detections[i].u_objectid=detectionList.List_Detections[i].u_ObjectID;
                            detectionMessage.list_detections[i].u_positivepredictivevalue=detectionList.List_Detections[i].u_PositivePredictiveValue;
                            detectionMessage.list_detections[i].u_sortindex=detectionList.List_Detections[i].u_SortIndex;   
                            posX=detectionList.List_Detections[i].f_Range*float(std::cos(detectionList.List_Detections[i].f_ElevationAngle))*float(std::cos(detectionList.List_Detections[i].f_AzimuthAngle));
                            posY=detectionList.List_Detections[i].f_Range*float(std::cos(detectionList.List_Detections[i].f_ElevationAngle))*float(std::sin(detectionList.List_Detections[i].f_AzimuthAngle));
                            posZ=detectionList.List_Detections[i].f_Range*float(std::sin(detectionList.List_Detections[i].f_ElevationAngle));
                           // if(posZ>0){
                                *iter_xD=posX;
                                *iter_yD=posY;
                                *iter_zD=posZ;
                           // }
                            std::cout<<"X Possition_Final: "<<posX<<std::endl;
                            std::cout<<"Y Possition_final: "<<posY<<std::endl;
                            std::cout<<"Z Possition_Final: "<<posZ<<std::endl;
                            
                            
                        }
                        
                        detectionMessage.list_invalidflags=detectionList.List_InvalidFlags;
                        detectionMessage.list_numofdetections=detectionList.List_NumOfDetections;
                        detectionMessage.list_radveldomain_max=detectionList.List_RadVelDomain_Max;
                        detectionMessage.list_radveldomain_min=detectionList.List_RadVelDomain_Min;
                        detectionMessage.aln_azimuthcorrection=detectionList.Aln_AzimuthCorrection;
                        detectionMessage.aln_elevationcorrection=detectionList.Aln_ElevationCorrection;
                        std::cout<<"SyncStatusD: "<<detectionList.Timestamp_SyncStatus<<std::endl;
                        pubDetect->publish(cloud_msgDetect);
                        detectionsPublisher->publish(detectionMessage);
   
                }             
                break;
            }
        }
    }

    public:
    std::string ars548_IP;
    std::string frame_ID;
    int ars548_Port;

    ars548_driver():Node("ars_548_driver_with_parameters"){
        //Parameter declaration so the user can change them
        this->declare_parameter("radarIP",DEFAULT_RADAR_IP);
        this->declare_parameter("radarPort",DEFAULT_RADAR_PORT);
        this->declare_parameter("frameID",DEFAULT_FRAME_ID);
        rclcpp::Clock clock;
        this->ars548_IP=this->get_parameter("radarIP").as_string();
        this->ars548_Port=this->get_parameter("radarPort").as_int();
        this->frame_ID=this->get_parameter("frameID").as_string();
        //Creation of their modifiers
        sensor_msgs::PointCloud2Modifier modifierObject(cloud_msgObj);
        sensor_msgs::PointCloud2Modifier modifierDetection(cloud_msgDetect);
        //Set fields and size of every PointCloud
        //Object Cloud
        modifierObject.setPointCloud2Fields(3,
            "x",1,sensor_msgs::msg::PointField::FLOAT32,
            "y",1,sensor_msgs::msg::PointField::FLOAT32,
            "z",1,sensor_msgs::msg::PointField::FLOAT32
        );
        modifierObject.resize(SIZE);
        //Detection Cloud
        modifierDetection.setPointCloud2Fields(3,
            "x",1,sensor_msgs::msg::PointField::FLOAT32,
            "y",1,sensor_msgs::msg::PointField::FLOAT32,
            "z",1,sensor_msgs::msg::PointField::FLOAT32
        );
        modifierDetection.resize(SIZE); 
        //handler subscription to the three callbacks so we can see if they have been changed
        readData(clock);
    }
};