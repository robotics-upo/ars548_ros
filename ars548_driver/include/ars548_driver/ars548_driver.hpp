/**
 * @file ars548_driver.hpp
 * 
 * @brief ars548_driver is a class that is used to obtain all of the data from the sensor, translates it and sends it to the user for later use.
 * It also copies part of the received data and sends it to Rviz for the visualization of the results. 
 */
#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cmath>
#include <memory>
#include <string>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>

#include "ars548_messages/Status.h"
#include "ars548_messages/DetectionList.h"
#include "ars548_messages/ObjectList.h"
#include "ars548_data.h"
/**
 * @brief Data obtained from the RadarSensors_Annex_AES548_IO SW 05.48.04.pdf 
 */
#define DEFAULT_RADAR_IP "224.0.2.2"
#define DEFAULT_RADAR_INTERFACE "10.13.1.166"
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
/**

 * @brief POINTCLOUD_HEIGHT = 1 because the pointcloud is unordered.
 */
#define POINTCLOUD_HEIGHT 1
/**
 * @brief This fields can be changed 
 */

#define SIZE 1000

class ARS548Driver{    
    
    private:
    char msgbuf[MSGBUFSIZE];
    int fd;
    int nbytes;

    std::string ars548_IP;
    std::string frame_ID;
    int ars548_Port;
    std::string radar_interface;

  std::unique_ptr<ros::NodeHandle> nh;
    
    sensor_msgs::PointCloud2 cloud_msgObj;
    sensor_msgs::PointCloud2 cloud_msgDetect;
    geometry_msgs::PoseArray cloud_Direction;

    sensor_msgs::PointCloud2Modifier modifierObject;
    sensor_msgs::PointCloud2Modifier modifierDetection;

    struct UDPStatus in_status;
    struct Object_List in_object_List;
    struct DetectionList in_detectionList;
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
    /**
     * @brief Changes the endianness of the object received 
     * @tparam v The object to be modified.
     * @return T. The object modified.
     */
    template<typename T>
    T ChangeEndianness(T v){
        T res = v;
        if (sizeof(T) == 2) {
            uint16_t i;
            std::memcpy(&i, &v, 2);
            i = __builtin_bswap16(i);
            std::memcpy(&res, &i, 2);
        } else if (sizeof(T) == 4) {
            uint32_t i;
            std::memcpy(&i, &v, 4);
            i = __builtin_bswap32(i);
            std::memcpy(&res, &i, 4);
        } else if (sizeof(T) == 8) {
            uint64_t i;
            std::memcpy(&i, &v, 8);
            i = __builtin_bswap64(i);
            std::memcpy(&res, &i, 8);
        } else if (sizeof(T) != 1) {
            uint8_t *pv = (uint8_t *)&v, *pr = (uint8_t *)&res;
            for (int i = 0; i < int(sizeof(T)); i++){
                pr[i] = pv[sizeof(T)-1-i];
            }
        }
        return res;
    }
    /**
     * @brief Changes the Endiannes of the status struct. 
     * 
     * @param status The UDPStatus struct that is going to be modified.
     * @return UDPStatus The modified struct. 
     */
    UDPStatus modifyStatus(UDPStatus status){
        status.Timestamp_Nanoseconds=ChangeEndianness(status.Timestamp_Nanoseconds);
        status.Timestamp_Seconds=ChangeEndianness(status.Timestamp_Seconds);
        status.Longitudinal=ChangeEndianness(status.Longitudinal);
        status.Lateral=ChangeEndianness(status.Lateral);
        status.Vertical=ChangeEndianness(status.Vertical);
        status.Yaw=ChangeEndianness(status.Yaw);
        status.Pitch=ChangeEndianness(status.Pitch);
        status.Length=ChangeEndianness(status.Length);
        status.Width=ChangeEndianness(status.Width);
        status.Height=ChangeEndianness(status.Height);
        status.Wheelbase=ChangeEndianness(status.Wheelbase);
        status.MaximumDistance=ChangeEndianness(status.MaximumDistance);
        status.SensorIPAddress_0=ChangeEndianness(status.SensorIPAddress_0);
        status.SensorIPAddress_1=ChangeEndianness(status.SensorIPAddress_1);
        return status;
    }
    /**
     * @brief Changes the endiannes of the Object_List struct
     *  
     * @param object_List The Object_List struct that is going to be modified.
     * @return Object_List The modified Struct.
     */
    void modifyObjectList(Object_List& object_List){
        object_List.CRC=ChangeEndianness(object_List.CRC);
        object_List.Length=ChangeEndianness(object_List.Length);
        object_List.SQC=ChangeEndianness(object_List.SQC);
        object_List.DataID=ChangeEndianness(object_List.DataID);
        object_List.Timestamp_Nanoseconds=ChangeEndianness(object_List.Timestamp_Nanoseconds);
        object_List.Timestamp_Seconds=ChangeEndianness(object_List.Timestamp_Seconds);
        object_List.EventDataQualifier=ChangeEndianness(object_List.EventDataQualifier);
        object_List.ObjectList_NumOfObjects=ChangeEndianness(object_List.ObjectList_NumOfObjects);
        if (object_List.ObjectList_NumOfObjects>50){
            object_List.ObjectList_NumOfObjects=50;
        }
        for(u_int32_t i = 0; i<object_List.ObjectList_NumOfObjects;++i){
            object_List.ObjectList_Objects[i].u_StatusSensor=ChangeEndianness(object_List.ObjectList_Objects[i].u_StatusSensor);
            object_List.ObjectList_Objects[i].u_ID=ChangeEndianness(object_List.ObjectList_Objects[i].u_ID);
            object_List.ObjectList_Objects[i].u_Age=ChangeEndianness(object_List.ObjectList_Objects[i].u_Age);
            object_List.ObjectList_Objects[i].u_Position_InvalidFlags=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_InvalidFlags);
            object_List.ObjectList_Objects[i].u_Position_X=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_X);
            object_List.ObjectList_Objects[i].u_Position_X_STD=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_X_STD);
            object_List.ObjectList_Objects[i].u_Position_Y=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_Y);
            object_List.ObjectList_Objects[i].u_Position_Y_STD=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_Y_STD);
            object_List.ObjectList_Objects[i].u_Position_Z=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_Z);
            object_List.ObjectList_Objects[i].u_Position_Z_STD=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_Z_STD);
            object_List.ObjectList_Objects[i].u_Position_CovarianceXY=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_CovarianceXY);
            object_List.ObjectList_Objects[i].u_Position_Orientation=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_Orientation);
            object_List.ObjectList_Objects[i].u_Position_Orientation_STD=ChangeEndianness(object_List.ObjectList_Objects[i].u_Position_Orientation_STD);
            object_List.ObjectList_Objects[i].u_Existence_Probability=ChangeEndianness(object_List.ObjectList_Objects[i].u_Existence_Probability);
            object_List.ObjectList_Objects[i].u_Existence_PPV=ChangeEndianness(object_List.ObjectList_Objects[i].u_Existence_PPV);
            object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X);
            object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X_STD);
            object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y);
            object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y_STD);
            object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_CovarianceXY=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_CovarianceXY);
            object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X);
            object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X_STD);
            object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y);
            object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y_STD);
            object_List.ObjectList_Objects[i].f_Dynamics_RelVel_CovarianceXY=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_CovarianceXY);
            object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X);
            object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_X_STD);
            object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y);
            object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_Y_STD);
            object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_CovarianceXY=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_AbsAccel_CovarianceXY);
            object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_X=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_X);
            object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_Y=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_Y);
            object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_X_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_X_STD);
            object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_Y_STD=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_Y_STD);
            object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_CovarianceXY=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_CovarianceXY);
            object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_Mean=ChangeEndianness(object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_Mean);
            object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_STD=ChangeEndianness(object_List.ObjectList_Objects[i].u_Dynamics_Orientation_Rate_STD);
            object_List.ObjectList_Objects[i].u_Shape_Length_Status=ChangeEndianness(object_List.ObjectList_Objects[i].u_Shape_Length_Status);
            object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean=ChangeEndianness(object_List.ObjectList_Objects[i].u_Shape_Length_Edge_Mean);
            object_List.ObjectList_Objects[i].u_Shape_Length_Edge_STD=ChangeEndianness(object_List.ObjectList_Objects[i].u_Shape_Length_Edge_STD);
            object_List.ObjectList_Objects[i].u_Shape_Width_Status=ChangeEndianness(object_List.ObjectList_Objects[i].u_Shape_Width_Status);
            object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean=ChangeEndianness(object_List.ObjectList_Objects[i].u_Shape_Width_Edge_Mean);
            object_List.ObjectList_Objects[i].u_Shape_Width_Edge_STD=ChangeEndianness(object_List.ObjectList_Objects[i].u_Shape_Width_Edge_STD);
        }
    }
    /**
     * @brief Changes the endianness of the DetectionList struct.
     * 
     * @param detectionList The DetectionList struct that is going to be modified.
     * @return DetectionList The modified struct.
     */
    void modifyDetectionList(DetectionList& detectionList){
        detectionList.CRC=ChangeEndianness(detectionList.CRC);
        detectionList.Length=ChangeEndianness(detectionList.Length);
        detectionList.SQC=ChangeEndianness(detectionList.SQC);
        detectionList.DataID=ChangeEndianness(detectionList.DataID);
        detectionList.Timestamp_Nanoseconds=ChangeEndianness(detectionList.Timestamp_Nanoseconds);
        detectionList.Timestamp_Seconds=ChangeEndianness(detectionList.Timestamp_Seconds);
        detectionList.EventDataQualifier=ChangeEndianness(detectionList.EventDataQualifier);
        detectionList.Origin_InvalidFlags=ChangeEndianness(detectionList.Origin_InvalidFlags);
        detectionList.Origin_Xpos=ChangeEndianness(detectionList.Origin_Xpos);
        detectionList.Origin_Xstd=ChangeEndianness(detectionList.Origin_Xstd);
        detectionList.Origin_Ypos=ChangeEndianness(detectionList.Origin_Ypos);
        detectionList.Origin_Ystd=ChangeEndianness(detectionList.Origin_Ystd);
        detectionList.Origin_Zpos=ChangeEndianness(detectionList.Origin_Zpos);
        detectionList.Origin_Zstd=ChangeEndianness(detectionList.Origin_Zstd);
        detectionList.Origin_Roll=ChangeEndianness(detectionList.Origin_Roll);
        detectionList.Origin_Rollstd=ChangeEndianness(detectionList.Origin_Rollstd);
        detectionList.Origin_Pitch=ChangeEndianness(detectionList.Origin_Pitch);
        detectionList.Origin_Pitchstd=ChangeEndianness(detectionList.Origin_Pitchstd);
        detectionList.Origin_Yaw=ChangeEndianness(detectionList.Origin_Yaw);
        detectionList.Origin_Yawstd=ChangeEndianness(detectionList.Origin_Yawstd);
        detectionList.List_NumOfDetections=ChangeEndianness(detectionList.List_NumOfDetections);
        detectionList.List_RadVelDomain_Min=ChangeEndianness(detectionList.List_RadVelDomain_Min);
        detectionList.List_RadVelDomain_Max=ChangeEndianness(detectionList.List_RadVelDomain_Max);
        detectionList.Aln_AzimuthCorrection=ChangeEndianness(detectionList.Aln_AzimuthCorrection);
        detectionList.Aln_ElevationCorrection=ChangeEndianness(detectionList.Aln_ElevationCorrection);
        for(uint64_t i=0; i<detectionList.List_NumOfDetections;i++){
            //Setting the detection data to littleEndian
            detectionList.List_Detections[i].f_AzimuthAngle=ChangeEndianness(detectionList.List_Detections[i].f_AzimuthAngle);
            detectionList.List_Detections[i].f_AzimuthAngleSTD=ChangeEndianness(detectionList.List_Detections[i].f_AzimuthAngleSTD);
            detectionList.List_Detections[i].f_ElevationAngle=ChangeEndianness(detectionList.List_Detections[i].f_ElevationAngle);
            detectionList.List_Detections[i].f_ElevationAngleSTD=ChangeEndianness(detectionList.List_Detections[i].f_ElevationAngleSTD);
            detectionList.List_Detections[i].f_Range=ChangeEndianness(detectionList.List_Detections[i].f_Range);
            detectionList.List_Detections[i].f_RangeSTD=ChangeEndianness(detectionList.List_Detections[i].f_RangeSTD);
            detectionList.List_Detections[i].f_RangeRate=ChangeEndianness(detectionList.List_Detections[i].f_RangeRate);
            detectionList.List_Detections[i].f_RangeRateSTD=ChangeEndianness(detectionList.List_Detections[i].f_RangeRateSTD);
            detectionList.List_Detections[i].u_MeasurementID=ChangeEndianness(detectionList.List_Detections[i].u_MeasurementID);
            detectionList.List_Detections[i].u_ObjectID=ChangeEndianness(detectionList.List_Detections[i].u_ObjectID);
            detectionList.List_Detections[i].u_SortIndex=ChangeEndianness(detectionList.List_Detections[i].u_SortIndex);
        }
    }
    /**
     * @brief Fills the Status Messsage.
     * 
     * @param statusMessage The Status message to be filled.
     * @param status The Status struct used to fill the message.
     * 
     */
    void fillStatusMessage(ars548_messages::Status &statusMessage, UDPStatus &status){
        statusMessage.cycletime=status.CycleTime;
        statusMessage.configurationcounter=status.ConfigurationCounter;
        statusMessage.frequencyslot=status.FrequencySlot;
        statusMessage.hcc=status.HCC;
        statusMessage.height=status.Height;
        statusMessage.lateral=status.Lateral;
        statusMessage.length=status.Length;
        statusMessage.longitudinal=status.Longitudinal;
        statusMessage.maximumdistance=status.MaximumDistance;
        statusMessage.pitch=status.Pitch;
        statusMessage.plugorientation=status.PlugOrientation;
        statusMessage.powersave_standstill=status.Powersave_Standstill;
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
    }
    /**
     * @brief Fills the ObjectList message.
     * 
     * @param objectMessage The object message to be filled.
     * @param object_List The Object_List struct used to fill the message.
     * @param clock The clock used to fill the timestamp of the message.
     * 
     */
    void fillMessageObject(ars548_messages::ObjectList &objectMessage,Object_List &object_List){
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
        objectMessage.header.stamp=ros::Time::now();
        for(u_int32_t i =0; i<object_List.ObjectList_NumOfObjects;++i){
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
            objectMessage.objectlist_objects[i].u_shape_width_edge_std=object_List.ObjectList_Objects[i].u_Shape_Width_Edge_STD;
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
            objectMessage.objectlist_objects[i].u_classification_unknown=object_List.ObjectList_Objects[i].u_Classification_Unknown;
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
        }
        
    }
    /**
     * @brief Fills the DetectionList message.
     * 
     * @param detectionMessage The DetectionList message to be filled.
     * @param detectionList The DetectionList struct used to fill the message.
     * @param clock The clock used to fill the timestamp of the message.
     */
    void fillDetectionMessage(ars548_messages::DetectionList &detectionMessage,DetectionList &detectionList){
        detectionMessage.header.frame_id=this->frame_ID;
        detectionMessage.header.stamp=ros::Time::now();
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
        for(uint64_t i=0; i<detectionList.List_NumOfDetections;i++){            
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
        }
        detectionMessage.list_invalidflags=detectionList.List_InvalidFlags;
        detectionMessage.list_numofdetections=detectionList.List_NumOfDetections;
        detectionMessage.list_radveldomain_max=detectionList.List_RadVelDomain_Max;
        detectionMessage.list_radveldomain_min=detectionList.List_RadVelDomain_Min;
        detectionMessage.aln_azimuthcorrection=detectionList.Aln_AzimuthCorrection;
        detectionMessage.aln_elevationcorrection=detectionList.Aln_ElevationCorrection;
    }
    /**
     * @brief Fills the PointCloud2 message. Used for visualization in Rviz2.
     * 
     * @param cloud_msg The PointCloud2 message that is going to be filled.
     */
    void fillCloudMessage(sensor_msgs::PointCloud2 &cloud_msg){
        cloud_msg.header=std_msgs::Header();
        cloud_msg.header.frame_id=this->frame_ID;
        cloud_msg.header.stamp=ros::Time::now();
        cloud_msg.is_dense=false;
        cloud_msg.is_bigendian=false;
        cloud_msg.height=POINTCLOUD_HEIGHT;
    }
    /**
     * @brief Fills the PoseArray message. Used for visualization in Rviz2.
     *
     * @param cloud_Direction The PoseArray message to be filled.
     * @param object_List The Object_List struct used to fill the message.
     * @param i The iterator used to fill the array of poses with the values of the points obtained from the struct.
     * @return PoseArray.msg. The message filled. 
     */
    void fillDirectionMessage(geometry_msgs::PoseArray &cloud_Direction,Object_List &object_List,u_int32_t i){
        tf::Quaternion q;
        float yaw;
        cloud_Direction.header = std_msgs::Header();
        cloud_Direction.header.frame_id=this->frame_ID;
        cloud_Direction.header.stamp=ros::Time::now();
        const auto& obj = object_List.ObjectList_Objects[i];
        cloud_Direction.poses[i].position.x = double(obj.u_Position_X);
        cloud_Direction.poses[i].position.y = double(obj.u_Position_Y);
        cloud_Direction.poses[i].position.z = double(obj.u_Position_Z);
        yaw = atan2(obj.f_Dynamics_RelVel_Y, obj.f_Dynamics_RelVel_X);   
        q.setRPY(0,0,yaw);
        cloud_Direction.poses[i].orientation.x=q.x();
        cloud_Direction.poses[i].orientation.y=q.y();
        cloud_Direction.poses[i].orientation.z=q.z();
        cloud_Direction.poses[i].orientation.w=q.w();
    }
    public:
    /**
     * @brief Reads the data received from the radar and sends it to the user and Rviz2.
     * 
     * @param clock The ROS2 clock used to fill some of the fields of the messages.
     * @return The status of the connection. If it returns 1, there is an error in the execution.
     */
    int readData(){
        //These are the publishers that send the data in a custom message
        auto statusPublisher = nh->advertise<ars548_messages::Status>("Status",10);
        auto objectPublisher = nh->advertise<ars548_messages::ObjectList>("ObjectList",10);
        auto detectionsPublisher = nh->advertise<ars548_messages::DetectionList>("DetectionList",10);
        //These are the publishers that send the data to Rviz2
        auto directionPublisher = nh->advertise<geometry_msgs::PoseArray>("DirectionVelocity",10);
        auto pubObj= nh->advertise<sensor_msgs::PointCloud2>("PointCloudObject",10);
        auto pubDetect=nh->advertise<sensor_msgs::PointCloud2>("PointCloudDetection",10);
        //Create messages for the publishers
        auto statusMessage=ars548_messages::Status();
        auto detectionMessage=ars548_messages::DetectionList();
        auto objectMessage=ars548_messages::ObjectList(); 
        
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
            close(fd);
            return 1;
        }

        // use setsockopt() to request that the kernel join a multicast group
        //
        struct ip_mreq mreq;
        mreq.imr_multiaddr.s_addr = inet_addr(this->ars548_IP.c_str());
        mreq.imr_interface.s_addr = inet_addr(this->radar_interface.c_str());

        if (
            setsockopt(
                fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq, sizeof(mreq)
            ) < 0
        ){
            perror("setsockopt");
            close(fd);
            return 1;
        }
        unsigned int addrlen = sizeof(addr);

        // Timeout for recvfrom
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
            perror("setsockopt SO_RCVTIMEO failed");
        }

        // FIX #6: use ros::ok() so the node stops cleanly on Ctrl+C / shutdown
        while (ros::ok())
        {
            nbytes = recvfrom(
                fd,
                msgbuf,
                MSGBUFSIZE,
                0,
                (struct sockaddr *) &addr,
                &addrlen
            );

            // FIX #6: continue on transient errors instead of killing the node
            if(nbytes<0){
                // Timeout
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                perror("recvfrom failed");
                continue;
            }
            switch (nbytes)
            {
            case STATUS_MESSAGE_PAYLOAD: {
                std::memcpy(&in_status, msgbuf, sizeof(UDPStatus));
                auto& status = in_status;
                status.ServiceID=ChangeEndianness(status.ServiceID);
                status.MethodID=ChangeEndianness(status.MethodID);
                status.PayloadLength=ChangeEndianness(status.PayloadLength);
                if(status.MethodID==STATUS_MESSAGE_METHOD_ID && status.PayloadLength==STATUS_MESSAGE_PDU_LENGTH){
                    status=modifyStatus(status);
                    fillStatusMessage(statusMessage,status);
                    statusPublisher.publish(statusMessage);
                }
                break;
            }
            case OBJECT_MESSAGE_PAYLOAD: {
                std::memcpy(&in_object_List, msgbuf, sizeof(Object_List));
                auto& object_List = in_object_List;
                object_List.ServiceID=ChangeEndianness(object_List.ServiceID);
                object_List.MethodID=ChangeEndianness(object_List.MethodID);
                object_List.PayloadLength=ChangeEndianness(object_List.PayloadLength);
                if(object_List.MethodID==OBJECT_MESSAGE_METHOD_ID && object_List.PayloadLength==OBJECT_MESSAGE_PDU_LENGTH){
                        modifyObjectList(object_List);
                        modifierObject.resize(object_List.ObjectList_NumOfObjects);
                        cloud_Direction.poses.resize(object_List.ObjectList_NumOfObjects);
                        fillMessageObject(objectMessage,object_List);
                        fillCloudMessage(cloud_msgObj);
                        // FIX #7: iterators created HERE, after resize(), inside the correct case
                        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msgObj,"x");
                        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msgObj,"y");
                        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msgObj,"z");
                        sensor_msgs::PointCloud2Iterator<float> iter_vx(cloud_msgObj,"vx");
                        sensor_msgs::PointCloud2Iterator<float> iter_vy(cloud_msgObj,"vy");
                        for(u_int32_t i =0; i<object_List.ObjectList_NumOfObjects;++i,++iter_x,++iter_y,++iter_z,++iter_vx,++iter_vy){
                            const auto& obj = object_List.ObjectList_Objects[i];
                            *iter_x=obj.u_Position_X;
                            *iter_y=obj.u_Position_Y;
                            *iter_z=obj.u_Position_Z;
                            *iter_vx=obj.f_Dynamics_AbsVel_X;
                            *iter_vy=obj.f_Dynamics_AbsVel_Y;
                            fillDirectionMessage(cloud_Direction,object_List,i);
                        }
                        pubObj.publish(cloud_msgObj);
                        directionPublisher.publish(cloud_Direction);
                        objectPublisher.publish(objectMessage);
                }
                break;
            }
            case DETECTION_MESSAGE_PAYLOAD: {
                float posX, posY, posZ;
                std::memcpy(&in_detectionList, msgbuf, sizeof(DetectionList));
                auto& detectionList = in_detectionList;
                detectionList.ServiceID=ChangeEndianness(detectionList.ServiceID);
                detectionList.MethodID=ChangeEndianness(detectionList.MethodID);
                detectionList.PayloadLength=ChangeEndianness(detectionList.PayloadLength);

                if(detectionList.MethodID==DETECTION_MESSAGE_METHOD_ID && detectionList.PayloadLength==DETECTION_MESSAGE_PDU_LENGTH){
                    modifyDetectionList(detectionList);
                    int valid_detections = 0;
                    for (uint64_t i = 0; i < detectionList.List_NumOfDetections; i++) {
                        if (detectionList.List_Detections[i].u_InvalidFlags == 0) {
                            valid_detections++;
                        }
                    }

                    modifierDetection.resize(static_cast<size_t>(valid_detections));
                    fillDetectionMessage(detectionMessage,detectionList);
                    fillCloudMessage(cloud_msgDetect);
                    
                    sensor_msgs::PointCloud2Iterator<float> iter_xD(cloud_msgDetect,"x");
                    sensor_msgs::PointCloud2Iterator<float> iter_yD(cloud_msgDetect,"y");
                    sensor_msgs::PointCloud2Iterator<float> iter_zD(cloud_msgDetect,"z");
                    sensor_msgs::PointCloud2Iterator<float> iter_vD(cloud_msgDetect,"v");
                    sensor_msgs::PointCloud2Iterator<float> iter_rD(cloud_msgDetect,"r");
                    sensor_msgs::PointCloud2Iterator<int8_t> iter_RCSD(cloud_msgDetect,"RCS");
                    sensor_msgs::PointCloud2Iterator<float> iter_azimuthD(cloud_msgDetect,"azimuth");
                    sensor_msgs::PointCloud2Iterator<float> iter_elevationD(cloud_msgDetect,"elevation");
                    
                    for(uint64_t i = 0; i < detectionList.List_NumOfDetections; i++){
                        const auto& detection = detectionList.List_Detections[i];
                        if (detection.u_InvalidFlags == 0) {
                            float cos_elev = std::cos(detection.f_ElevationAngle);
                            float sin_elev = std::sin(detection.f_ElevationAngle);
                            float cos_azi = std::cos(detection.f_AzimuthAngle);
                            float sin_azi = std::sin(detection.f_AzimuthAngle);
                            posX = detection.f_Range*cos_elev*cos_azi;
                            posY = detection.f_Range*cos_elev*sin_azi;
                            posZ = detection.f_Range*sin_elev;
                            *iter_xD = posX;
                            *iter_yD = posY;
                            *iter_zD = posZ;
                            *iter_rD = detection.f_Range;
                            *iter_vD = detection.f_RangeRate;
                            *iter_RCSD = detection.s_RCS;
                            *iter_azimuthD = detection.f_AzimuthAngle;
                            *iter_elevationD = detection.f_ElevationAngle;
                            
                            ++iter_xD; ++iter_yD; ++iter_zD;
                            ++iter_vD; ++iter_rD; ++iter_RCSD;
                            ++iter_azimuthD; ++iter_elevationD;
                        }
                    }
                    pubDetect.publish(cloud_msgDetect);
                    detectionsPublisher.publish(detectionMessage);
                }
                break;
            }
            }
        }
        return 0;
    }


    /**
     * @brief  ars548_driver Node.
     */
  ARS548Driver():modifierObject(cloud_msgObj),modifierDetection(cloud_msgDetect){
    nh.reset(new ros::NodeHandle("~"));

    nh->param("radarIP",        ars548_IP,       static_cast<std::string>(DEFAULT_RADAR_IP));
    nh->param("radarPort",      ars548_Port,     DEFAULT_RADAR_PORT);
    nh->param("frameID",        frame_ID,        static_cast<std::string>(DEFAULT_FRAME_ID));
    nh->param("radarInterface", radar_interface, static_cast<std::string>(DEFAULT_RADAR_INTERFACE));

    //Creation of their modifiers

    //Set fields and size of every PointCloud
    //Object Cloud
    modifierObject.setPointCloud2Fields(5,
                                        "x",1,sensor_msgs::PointField::FLOAT32,
                                        "y",1,sensor_msgs::PointField::FLOAT32,
                                        "z",1,sensor_msgs::PointField::FLOAT32,
                                        "vx",1,sensor_msgs::PointField::FLOAT32,
                                        "vy",1,sensor_msgs::PointField::FLOAT32
                                        );
    modifierObject.reserve(SIZE);
    modifierObject.clear();

    //Detection Cloud
    modifierDetection.setPointCloud2Fields(8,
                                           "x",1,sensor_msgs::PointField::FLOAT32,
                                           "y",1,sensor_msgs::PointField::FLOAT32,
                                           "z",1,sensor_msgs::PointField::FLOAT32,
                                           "v",1,sensor_msgs::PointField::FLOAT32,
                                           "r",1,sensor_msgs::PointField::FLOAT32,
                                           "RCS",1,sensor_msgs::PointField::INT8,
                                           "azimuth",1,sensor_msgs::PointField::FLOAT32,
                                           "elevation",1,sensor_msgs::PointField::FLOAT32
                                           );
    modifierDetection.reserve(SIZE);
    modifierDetection.clear();
    cloud_Direction.poses.reserve(SIZE);
  }

  ~ARS548Driver() {
      if (fd >= 0) {
          close(fd);
      }
  }
    
};
