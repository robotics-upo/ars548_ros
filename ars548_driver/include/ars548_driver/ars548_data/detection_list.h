#pragma once

#include "detection.h"
#include <cstdint>
#include <ars548_driver/byteswap.hpp>

#pragma pack(4)

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

    inline void changeEndianness();
};

/**
     * @brief Changes the endianness of the DetectionList struct.
     * 
     * @return DetectionList The modified struct.
*/
void DetectionList::changeEndianness(){
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