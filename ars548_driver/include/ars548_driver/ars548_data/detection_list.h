#pragma once

#include "detection.h"
#include <cstdint>
#include <ars548_driver/util/byteswap.hpp>
#include <ars548_messages/msg/detection_list.hpp>

#pragma pack(1)

#define ARS548_MAX_DETECTIONS 800
#define DETECTION_MESSAGE_METHOD_ID 336
#define DETECTION_MESSAGE_PDU_LENGTH 35328
#define DETECTION_MESSAGE_PAYLOAD 35336

struct DetectionList{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    int64_t empty1;//Because the data starts at bit 71
    uint64_t CRC;
    uint32_t Length;
    uint32_t SQC;
    uint32_t DataID;
    uint32_t Timestamp_Nanoseconds;
    uint32_t Timestamp_Seconds;
    uint8_t Timestamp_SyncStatus;
    uint32_t EventDataQualifier;
    uint8_t ExtendedQualifier;
    uint16_t Origin_InvalidFlags;
    float Origin_Xpos;
    float Origin_Xstd;
    float Origin_Ypos;
    float Origin_Ystd;
    float Origin_Zpos;
    float Origin_Zstd;
    float Origin_Roll;
    float Origin_Rollstd;
    float Origin_Pitch;
    float Origin_Pitchstd;
    float Origin_Yaw;
    float Origin_Yawstd;
    uint8_t List_InvalidFlags;
    struct Detection List_Detections[ARS548_MAX_DETECTIONS];
    float List_RadVelDomain_Min;
    float List_RadVelDomain_Max;
    uint32_t List_NumOfDetections;
    float Aln_AzimuthCorrection;
    float Aln_ElevationCorrection;
    uint8_t Aln_Status;

    //! @brief Checks for validity of the message (checks method ID and PayloadLength)
    inline bool isValid() const {
        return MethodID == DETECTION_MESSAGE_METHOD_ID && PayloadLength == DETECTION_MESSAGE_PDU_LENGTH;
    }

    inline void changeEndianness();

    inline ars548_messages::msg::DetectionList toMsg(const std::string &frame_ID, bool override_stamp = true);

    inline void fillDetectionCloud(sensor_msgs::msg::PointCloud2 &cloud_msg, sensor_msgs::PointCloud2Modifier &modifierDetection,
                                  const std::string &frame_id, bool override_stamp = true);
};

/**
     * @brief Changes the endianness of the DetectionList struct.
     * 
     * @return DetectionList The modified struct.
*/
void DetectionList::changeEndianness() {
    ServiceID = byteswap(ServiceID);
    MethodID = byteswap(MethodID);
    PayloadLength = byteswap(PayloadLength);
    CRC = byteswap(CRC);
    Length = byteswap(Length);
    SQC = byteswap(SQC);
    DataID = byteswap(DataID);
    Timestamp_Nanoseconds = byteswap(Timestamp_Nanoseconds);
    Timestamp_Seconds = byteswap(Timestamp_Seconds);
    EventDataQualifier = byteswap(EventDataQualifier);
    Origin_InvalidFlags = byteswap(Origin_InvalidFlags);
    Origin_Xpos = byteswap(Origin_Xpos);
    Origin_Xstd = byteswap(Origin_Xstd);
    Origin_Ypos = byteswap(Origin_Ypos);
    Origin_Ystd = byteswap(Origin_Ystd);
    Origin_Zpos = byteswap(Origin_Zpos);
    Origin_Zstd = byteswap(Origin_Zstd);
    Origin_Roll = byteswap(Origin_Roll);
    Origin_Rollstd = byteswap(Origin_Rollstd);
    Origin_Pitch = byteswap(Origin_Pitch);
    Origin_Pitchstd = byteswap(Origin_Pitchstd);
    Origin_Yaw = byteswap(Origin_Yaw);
    Origin_Yawstd = byteswap(Origin_Yawstd);
    List_NumOfDetections = byteswap(List_NumOfDetections);
    List_RadVelDomain_Min = byteswap(List_RadVelDomain_Min);
    List_RadVelDomain_Max = byteswap(List_RadVelDomain_Max);
    Aln_AzimuthCorrection = byteswap(Aln_AzimuthCorrection);
    Aln_ElevationCorrection = byteswap(Aln_ElevationCorrection);
    for(uint64_t i=0; i<List_NumOfDetections;i++){
        //Setting the detection data to littleEndian
        List_Detections[i].changeEndianness();        
    }
}

#pragma pack(4)

inline ars548_messages::msg::DetectionList DetectionList::toMsg(const std::string &frame_ID, bool override_stamp) {
    ars548_messages::msg::DetectionList detectionMessage;

    detectionMessage.header.frame_id = frame_ID;
    if (override_stamp) {
        rclcpp::Clock clock;
        detectionMessage.header.stamp = clock.now();
    } else {
        detectionMessage.header.stamp.sec = Timestamp_Seconds;
        detectionMessage.header.stamp.nanosec = Timestamp_Nanoseconds;
    }
    detectionMessage.aln_status = Aln_Status;
    detectionMessage.crc = CRC;
    detectionMessage.dataid = DataID;
    detectionMessage.eventdataqualifier = EventDataQualifier;
    detectionMessage.extendedqualifier = ExtendedQualifier;
    detectionMessage.length = Length;
    detectionMessage.origin_invalidflags = Origin_InvalidFlags;
    detectionMessage.origin_pitch = Origin_Pitch;
    detectionMessage.origin_pitchstd = Origin_Pitchstd;
    detectionMessage.origin_roll = Origin_Roll;
    detectionMessage.origin_rollstd = Origin_Rollstd;
    detectionMessage.origin_xpos = Origin_Xpos;
    detectionMessage.origin_xstd = Origin_Xstd;
    detectionMessage.origin_yaw = Origin_Yaw;
    detectionMessage.origin_yawstd = Origin_Yawstd;
    detectionMessage.origin_ypos = Origin_Ypos;
    detectionMessage.origin_ystd = Origin_Ystd;
    detectionMessage.origin_zpos = Origin_Zpos;
    detectionMessage.origin_zstd = Origin_Zstd;
    detectionMessage.sqc = SQC;
    detectionMessage.timestamp_nanoseconds = Timestamp_Nanoseconds;
    detectionMessage.timestamp_seconds = Timestamp_Seconds;
    detectionMessage.timestamp_syncstatus = Timestamp_SyncStatus;

    if (List_NumOfDetections > ARS548_MAX_DETECTIONS) {
        List_NumOfDetections = ARS548_MAX_DETECTIONS;
    }
    detectionMessage.list_numofdetections = List_NumOfDetections;

    for(uint32_t i=0; i < List_NumOfDetections;i++) {
        detectionMessage.list_detections[i] = List_Detections[i].toMsg();
    }

    return detectionMessage;
}

/**
 * @brief Fills the PointCloud2 message. Can be for visualization in Rviz2.
 * 
 * @param cloud_msg The PointCloud2 message that is going to be filled.
 */
#define DETECTION_LIST_POINTCLOUD_HEIGHT 1
inline void DetectionList::fillDetectionCloud(sensor_msgs::msg::PointCloud2 &cloud_msg, sensor_msgs::PointCloud2Modifier &modifierDetection, 
                                           const std::string &frame_id, bool override_stamp) {
    cloud_msg.header=std_msgs::msg::Header();
    cloud_msg.header.frame_id = frame_id;
    modifierDetection.resize(static_cast<size_t>(List_NumOfDetections));
    if (override_stamp) {
        rclcpp::Clock clock;
        cloud_msg.header.stamp = clock.now();
    }
    cloud_msg.is_dense=false;
    cloud_msg.is_bigendian=false;
    cloud_msg.height = DETECTION_LIST_POINTCLOUD_HEIGHT;

    sensor_msgs::PointCloud2Iterator<float> iter_xD(cloud_msg,"x");
    sensor_msgs::PointCloud2Iterator<float> iter_yD(cloud_msg,"y");
    sensor_msgs::PointCloud2Iterator<float> iter_zD(cloud_msg,"z");
    sensor_msgs::PointCloud2Iterator<float> iter_vD(cloud_msg,"v");
    sensor_msgs::PointCloud2Iterator<float> iter_rD(cloud_msg,"r");
    sensor_msgs::PointCloud2Iterator<int8_t> iter_RCSD(cloud_msg,"RCS");
    sensor_msgs::PointCloud2Iterator<float> iter_azimuthD(cloud_msg, "azimuth");
    sensor_msgs::PointCloud2Iterator<float> iter_elevationD(cloud_msg,"elevation");

    for(uint32_t i = 0; i < List_NumOfDetections;i++,++iter_xD,++iter_yD,++iter_zD,
                                                                ++iter_vD, ++iter_rD, ++iter_RCSD,
                                                                ++iter_azimuthD, ++iter_elevationD){
        auto posX = List_Detections[i].f_Range*float(std::cos(List_Detections[i].f_ElevationAngle))*float(std::cos(List_Detections[i].f_AzimuthAngle));
        auto posY = List_Detections[i].f_Range*float(std::cos(List_Detections[i].f_ElevationAngle))*float(std::sin(List_Detections[i].f_AzimuthAngle));
        auto posZ = List_Detections[i].f_Range*float(std::sin(List_Detections[i].f_ElevationAngle));
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Detection possition \n x: %f\n y: %f\n z:%f.", posX, posY, posZ);
        *iter_xD = posX;
        *iter_yD = posY;
        *iter_zD = posZ;
        *iter_rD = List_Detections[i].f_Range;
        *iter_vD = List_Detections[i].f_RangeRate;
        *iter_RCSD = List_Detections[i].s_RCS;
        *iter_azimuthD = List_Detections[i].f_AzimuthAngle;
        *iter_elevationD = List_Detections[i].f_ElevationAngle;
    }
}
    