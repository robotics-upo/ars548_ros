#pragma once
#include <cstdint>
#include <ars548_driver/byteswap.hpp>
#include "object.h"

#define ARS548_MAX_OBJECTS 100

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

    inline void changeEndianness();
};

void ObjectList::changeEndianness() {
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