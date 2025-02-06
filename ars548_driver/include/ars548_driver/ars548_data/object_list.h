#pragma once
#include <cstdint>
#include <ars548_driver/util/byteswap.hpp>
#include "object.h"
#include <ars548_messages/msg/object_list.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#define ARS548_OBJECT_POINTCLOUD_HEIGHT 1
#define ARS548_MAX_OBJECTS 50

#define OBJECT_MESSAGE_METHOD_ID 329
#define OBJECT_MESSAGE_PDU_LENGTH 9393
#define OBJECT_MESSAGE_PAYLOAD 9401


#pragma pack(1)

struct ObjectList{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    uint64_t empty1;//Because the data starts at bit 71
    uint64_t CRC;
    uint32_t Length;
    uint32_t SQC;
    uint32_t DataID;
    uint32_t Timestamp_Nanoseconds;
    uint32_t Timestamp_Seconds;
    uint8_t Timestamp_SyncStatus;
    uint32_t EventDataQualifier;
    uint8_t ExtendedQualifier;
    uint8_t ObjectList_NumOfObjects;
    struct Object ObjectList_Objects[ARS548_MAX_OBJECTS];

    //! @brief Checks for validity of the message (checks method ID and PayloadLength)
    inline bool isValid() const {
        return MethodID == OBJECT_MESSAGE_METHOD_ID && PayloadLength == OBJECT_MESSAGE_PDU_LENGTH;
    }

    inline void changeEndianness();

    inline ars548_messages::msg::ObjectList toMsg(const std::string &frame_ID, bool override_stamp = true);

    inline void fillObjectCloud(sensor_msgs::msg::PointCloud2 &cloud_msg, sensor_msgs::PointCloud2Modifier &modifierObject,
                                  const std::string &frame_id, bool override_stamp = true);

    geometry_msgs::msg::PoseArray getDirectionMessage(const std::string &frame_id, bool override_stamp = true);
};

void ObjectList::changeEndianness() {
    ServiceID = byteswap(ServiceID);
    MethodID = byteswap(MethodID);
    PayloadLength = byteswap(PayloadLength);
    CRC=byteswap(CRC);
    Length=byteswap(Length);
    SQC=byteswap(SQC);
    DataID=byteswap(DataID);
    Timestamp_Nanoseconds=byteswap(Timestamp_Nanoseconds);
    Timestamp_Seconds=byteswap(Timestamp_Seconds);
    EventDataQualifier=byteswap(EventDataQualifier);
    ObjectList_NumOfObjects=byteswap(ObjectList_NumOfObjects);
    if (ObjectList_NumOfObjects>ARS548_MAX_OBJECTS) {
        ObjectList_NumOfObjects=ARS548_MAX_OBJECTS;
    }
    for(uint32_t i = 0; i<ObjectList_NumOfObjects;++i) {
        ObjectList_Objects[i].changeEndianness();
    }
}

#pragma pack(4)

inline ars548_messages::msg::ObjectList ObjectList::toMsg(const std::string &frame_ID, bool override_stamp) {
    ars548_messages::msg::ObjectList objectMessage;

    objectMessage.crc = CRC;
    objectMessage.length = Length;
    objectMessage.sqc = SQC;
    objectMessage.timestamp_nanoseconds = Timestamp_Nanoseconds;
    objectMessage.timestamp_seconds = Timestamp_Seconds;
    objectMessage.eventdataqualifier = EventDataQualifier;
    objectMessage.extendedqualifier = ExtendedQualifier;
    objectMessage.dataid = DataID;

    if (ObjectList_NumOfObjects > ARS548_MAX_OBJECTS) {
        ObjectList_NumOfObjects = ARS548_MAX_OBJECTS;
    }
    objectMessage.objectlist_numofobjects = ObjectList_NumOfObjects;

    for(u_int32_t i =0; i< ObjectList_NumOfObjects;++i) {
        objectMessage.objectlist_objects[i] = ObjectList_Objects[i].toMsg();
    }

    objectMessage.objectlist_numofobjects = ObjectList_NumOfObjects;
    objectMessage.timestamp_syncstatus = Timestamp_SyncStatus;
    
    objectMessage.header.frame_id = frame_ID;
    if (override_stamp) {
        rclcpp::Clock clock;
        objectMessage.header.stamp = clock.now();
    } else {
        objectMessage.header.stamp.sec = Timestamp_Seconds;
        objectMessage.header.stamp.nanosec = Timestamp_Nanoseconds;
    }

    
    return objectMessage;
}

inline void ObjectList::fillObjectCloud(sensor_msgs::msg::PointCloud2 &cloud_msg, sensor_msgs::PointCloud2Modifier &modifierObject,
                            const std::string &frame_id, bool override_stamp) {
    
    cloud_msg.header.frame_id = frame_id;
    if (override_stamp) {
        rclcpp::Clock clock;
        cloud_msg.header.stamp = clock.now();
    } else {
        cloud_msg.header.stamp.sec = Timestamp_Seconds;
        cloud_msg.header.stamp.nanosec = Timestamp_Nanoseconds;
    }

    //Object Iterators
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg,"x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg,"y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg,"z");
    sensor_msgs::PointCloud2Iterator<float> iter_vx(cloud_msg,"vx");
    sensor_msgs::PointCloud2Iterator<float> iter_vy(cloud_msg,"vy");

    // Metadata
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;
    cloud_msg.height = ARS548_OBJECT_POINTCLOUD_HEIGHT;

    if (ObjectList_NumOfObjects > ARS548_MAX_OBJECTS) {
        ObjectList_NumOfObjects = ARS548_MAX_OBJECTS;
    }
    modifierObject.resize(ObjectList_NumOfObjects);

    for(u_int32_t i = 0; i < ObjectList_NumOfObjects;++i, ++iter_x, ++iter_y, ++iter_z,
                                                     ++iter_vx, ++iter_vy) {
        
        *iter_x = ObjectList_Objects[i].u_Position_X;
        *iter_y = ObjectList_Objects[i].u_Position_Y;
        *iter_z = ObjectList_Objects[i].u_Position_Z;
        *iter_vx = ObjectList_Objects[i].f_Dynamics_AbsVel_X;
        *iter_vy = ObjectList_Objects[i].f_Dynamics_AbsVel_Y;
        
    }


}

geometry_msgs::msg::PoseArray ObjectList::getDirectionMessage(const std::string &frame_id, bool override_stamp){
    geometry_msgs::msg::PoseArray cloud_Direction;
    tf2::Quaternion q;
    float yaw;
    cloud_Direction.header = std_msgs::msg::Header();
    cloud_Direction.header.frame_id = frame_id;
    if (override_stamp) {
        rclcpp::Clock clock;
        cloud_Direction.header.stamp = clock.now();
    } else {
        cloud_Direction.header.stamp.sec = Timestamp_Seconds;
        cloud_Direction.header.stamp.nanosec = Timestamp_Nanoseconds;
    }
    cloud_Direction.poses.resize(ObjectList_NumOfObjects);
    for(u_int32_t i = 0; i < ObjectList_NumOfObjects; ++i) {
        cloud_Direction.poses[i].position.x = double(ObjectList_Objects[i].u_Position_X);
        cloud_Direction.poses[i].position.y = double(ObjectList_Objects[i].u_Position_Y);
        cloud_Direction.poses[i].position.z = double(ObjectList_Objects[i].u_Position_Z);
        yaw = atan2(ObjectList_Objects[i].f_Dynamics_RelVel_Y,ObjectList_Objects[i].f_Dynamics_RelVel_X);   
        q.setRPY(0,0,yaw);
        cloud_Direction.poses[i].orientation.x=q.x();
        cloud_Direction.poses[i].orientation.y=q.y();
        cloud_Direction.poses[i].orientation.z=q.z();
        cloud_Direction.poses[i].orientation.w=q.w();
    }
    return cloud_Direction;
}