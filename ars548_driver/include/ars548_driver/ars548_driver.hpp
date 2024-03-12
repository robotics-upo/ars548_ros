/**
 * @file ars548_driver.hpp
 * 
 * @brief ars548_driver is a class that is used to obtain all of the data from the sensor, translates it and sends it to the user for later use.
 * It also copies part of the received data and sends it to Rviz for the visualization of the results. 
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
/**
 * @brief POINTCLOUD_HEIGHT = 1 because the pointcloud is unordered.
 */
#define POINTCLOUD_HEIGHT 1
/**
 * @brief This fields can be changed 
 */

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
    /**
     * @brief Changes the endianness of the object received 
     * @tparam v The object to be modified.
     * @return T. The object modified.
     */
    template<typename T>
    T ChangeEndianness(T v){
        T r;
        uint8_t *pv = (uint8_t *)&v, *pr = (uint8_t *)&r;
        for (int i = 0;i <int(sizeof(T)); i++){
            pr[i]=pv[sizeof(T)-1-i];
        }
        return r;
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
        status.MaximunDistance=ChangeEndianness(status.MaximunDistance);
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
    Object_List modifyObjectList(Object_List object_List){
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
        for(u_int32_t i =0; i<object_List.ObjectList_NumOfObjects;++i){
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
            object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_X=ChangeEndianness(object_List.ObjectList_Objects[i].f_Dynamics_RelAccel_Y_STD);
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
        return object_List;
    }
    /**
     * @brief Changes the endianness of the DetectionList struct.
     * 
     * @param detectionList The DetectionList struct that is going to be modified.
     * @return DetectionList The modified struct.
     */
    DetectionList modifyDetectionList(DetectionList detectionList){
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
        return detectionList;
    }
    /**
     * @brief Fills the Status Messsage.
     * 
     * @param statusMessage The Status message to be filled.
     * @param status The Status struct used to fill the message.
     * 
     */
    void fillStatusMessage(ars548_messages::msg::Status &statusMessage, UDPStatus status){
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
    }
    /**
     * @brief Fills the ObjectList message.
     * 
     * @param objectMessage The object message to be filled.
     * @param object_List The Object_List struct used to fill the message.
     * @param clock The clock used to fill the timestamp of the message.
     * 
     */
    void fillMessageObject(ars548_messages::msg::ObjectList &objectMessage,Object_List object_List,rclcpp::Clock clock){
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
        }
        
    }
    /**
     * @brief Fills the DetectionList message.
     * 
     * @param detectionMessage The DetectionList message to be filled.
     * @param detectionList The DetectionList struct used to fill the message.
     * @param clock The clock used to fill the timestamp of the message.
     */
    void fillDetectionMessage(ars548_messages::msg::DetectionList &detectionMessage,DetectionList detectionList,rclcpp::Clock clock){
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
    void fillCloudMessage(sensor_msgs::msg::PointCloud2 &cloud_msg){
        cloud_msg.header=std_msgs::msg::Header();
        cloud_msg.header.frame_id=this->frame_ID;
        std::chrono::time_point<std::chrono::system_clock> now=std::chrono::system_clock::now();
        auto duration=now.time_since_epoch();
        cloud_msg.header.stamp.nanosec=std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        cloud_msg.header.stamp.sec=std::chrono::duration_cast<std::chrono::seconds>(duration).count();
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
    void fillDirectionMessage(geometry_msgs::msg::PoseArray &cloud_Direction,Object_List object_List,u_int32_t i){
        tf2::Quaternion q;
        float yaw;
        cloud_Direction.header = std_msgs::msg::Header();
        cloud_Direction.header.frame_id=this->frame_ID;
        cloud_Direction.header.stamp.nanosec=std::chrono::nanoseconds().count();
        cloud_Direction.header.stamp.sec=std::chrono::seconds().count();
        cloud_Direction.poses[i].position.x = double(object_List.ObjectList_Objects[i].u_Position_X);
        cloud_Direction.poses[i].position.y = double(object_List.ObjectList_Objects[i].u_Position_Y);
        cloud_Direction.poses[i].position.z = double(object_List.ObjectList_Objects[i].u_Position_Z);
        yaw = atan2(object_List.ObjectList_Objects[i].f_Dynamics_RelVel_Y,object_List.ObjectList_Objects[i].f_Dynamics_RelVel_X);   
        q.setRPY(0,0,yaw);
        cloud_Direction.poses[i].orientation.x=q.x();
        cloud_Direction.poses[i].orientation.y=q.y();
        cloud_Direction.poses[i].orientation.z=q.z();
        cloud_Direction.poses[i].orientation.w=q.w();
    }
    /**
     * @brief Reads the data received from the radar and sends it to the user and Rviz2.
     * 
     * @param clock The ROS2 clock used to fill some of the fields of the messages.
     * @return The status of the connection. If it returns 1, there is an error in the execution.
     */
    int readData(rclcpp::Clock clock){
        auto node=rclcpp::Node::make_shared("publisherObj");
        //These are the publishers that send the data in a custom message
        auto statusPublisher=node->create_publisher<ars548_messages::msg::Status>("Status",10);
        auto objectPublisher=node->create_publisher<ars548_messages::msg::ObjectList>("ObjectList",10);
        auto detectionsPublisher=node->create_publisher<ars548_messages::msg::DetectionList>("DetectionList",10);
        //These are the publishers that send the data to Rviz2
        auto directionPublisher=node->create_publisher<geometry_msgs::msg::PoseArray>("DirectionVelocity",10);
        auto pubObj= node->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudObject",10);
        auto pubDetect=node->create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudDetection",10);
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
            sensor_msgs::PointCloud2Iterator<float> iter_vx(cloud_msgObj,"vx");
            sensor_msgs::PointCloud2Iterator<float> iter_vy(cloud_msgObj,"vy");


            if(nbytes<0){
                perror("Failed attempt of getting data");
                return 1;
            }
            switch (nbytes)
            {
            case STATUS_MESSAGE_PAYLOAD:
                struct UDPStatus status;
                status = *((struct UDPStatus *)msgbuf);
                status.ServiceID=ChangeEndianness(status.ServiceID);
                status.MethodID=ChangeEndianness(status.MethodID);
                status.PayloadLength=ChangeEndianness(status.PayloadLength);
                if(status.MethodID==STATUS_MESSAGE_METHOD_ID && status.PayloadLength==STATUS_MESSAGE_PDU_LENGTH){
                    status=modifyStatus(status);
                    fillStatusMessage(statusMessage,status);
                    statusPublisher->publish(statusMessage);
                }
                break;
            case OBJECT_MESSAGE_PAYLOAD:
                struct Object_List object_List;
                object_List=*((struct Object_List *)msgbuf);
                object_List.ServiceID=ChangeEndianness(object_List.ServiceID);
                object_List.MethodID=ChangeEndianness(object_List.MethodID);
                object_List.PayloadLength=ChangeEndianness(object_List.PayloadLength);
                //Setting the ars548_messages
                if(object_List.MethodID==OBJECT_MESSAGE_METHOD_ID && object_List.PayloadLength==OBJECT_MESSAGE_PDU_LENGTH){
                     //Changes all of the > 8bit data to little endian
                        object_List=modifyObjectList(object_List);
                        modifierObject.resize(object_List.ObjectList_NumOfObjects);
                        cloud_Direction.poses.resize(object_List.ObjectList_NumOfObjects);
                        fillMessageObject(objectMessage,object_List,clock);
                        
                        //Changes all of the > 8bit data to little endian inside the 50 element array
                        fillCloudMessage(cloud_msgObj);
                        for(u_int32_t i =0; i<object_List.ObjectList_NumOfObjects;++i,++iter_x,++iter_y,++iter_z,++iter_vx,++iter_vy){
                           // AbsVel=3.6*sqrt(pow(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X,2)+pow(object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y,2));
                            *iter_x=object_List.ObjectList_Objects[i].u_Position_X;
                            *iter_y=object_List.ObjectList_Objects[i].u_Position_Y;
                            *iter_z=object_List.ObjectList_Objects[i].u_Position_Z;
                            *iter_vx=object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_X;
                            *iter_vy=object_List.ObjectList_Objects[i].f_Dynamics_AbsVel_Y;
                            //To show the direction of the moving object
                            fillDirectionMessage(cloud_Direction,object_List,i);
                        } 

                        pubObj->publish(cloud_msgObj);
                        directionPublisher->publish(cloud_Direction);
                        objectPublisher->publish(objectMessage);   
                }
                break;
            case DETECTION_MESSAGE_PAYLOAD:
                float posX, posY,posZ;
                
                struct DetectionList detectionList;
                detectionList=*((struct DetectionList *)msgbuf);
                detectionList.ServiceID=ChangeEndianness(detectionList.ServiceID);
                detectionList.MethodID=ChangeEndianness(detectionList.MethodID);
                detectionList.PayloadLength=ChangeEndianness(detectionList.PayloadLength);
                
                fillCloudMessage(cloud_msgDetect);
                
                if(detectionList.MethodID==DETECTION_MESSAGE_METHOD_ID && detectionList.PayloadLength==DETECTION_MESSAGE_PDU_LENGTH){
                    detectionList=modifyDetectionList(detectionList);
                    modifierDetection.resize(static_cast<size_t> (detectionList.List_NumOfDetections));
                    fillDetectionMessage(detectionMessage,detectionList,clock);
                        
                        //Changes all of the > 8bit data to little endian inside the 800 elements array
                        for(uint64_t i=0; i<detectionList.List_NumOfDetections;i++,++iter_xD,++iter_yD,++iter_zD){
                            posX=detectionList.List_Detections[i].f_Range*float(std::cos(detectionList.List_Detections[i].f_ElevationAngle))*float(std::cos(detectionList.List_Detections[i].f_AzimuthAngle));
                            posY=detectionList.List_Detections[i].f_Range*float(std::cos(detectionList.List_Detections[i].f_ElevationAngle))*float(std::sin(detectionList.List_Detections[i].f_AzimuthAngle));
                            posZ=detectionList.List_Detections[i].f_Range*float(std::sin(detectionList.List_Detections[i].f_ElevationAngle));
                            *iter_xD=posX;
                            *iter_yD=posY;
                            *iter_zD=posZ;
                        }
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
   
    /**
     * @brief  ars548_driver Node. Used to try the driver. 
     */
    ars548_driver():Node("ars_548_driver_with_parameters"),modifierObject(cloud_msgObj),modifierDetection(cloud_msgDetect){
        //Parameter declaration so the user can change them
        this->declare_parameter("radarIP",DEFAULT_RADAR_IP);
        this->declare_parameter("radarPort",DEFAULT_RADAR_PORT);
        this->declare_parameter("frameID",DEFAULT_FRAME_ID);
        rclcpp::Clock clock;
        this->ars548_IP=this->get_parameter("radarIP").as_string();
        this->ars548_Port=this->get_parameter("radarPort").as_int();
        this->frame_ID=this->get_parameter("frameID").as_string();
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
     
        modifierObject.reserve(SIZE);
        modifierObject.clear();
        //Detection Cloud
        modifierDetection.setPointCloud2Fields(3,
            "x",1,sensor_msgs::msg::PointField::FLOAT32,
            "y",1,sensor_msgs::msg::PointField::FLOAT32,
            "z",1,sensor_msgs::msg::PointField::FLOAT32
        );
        modifierDetection.reserve(SIZE);
        modifierDetection.clear();
        cloud_Direction.poses.reserve(SIZE);
        //handler subscription to the three callbacks so we can see if they have been changed
        readData(clock);

    }
    
};