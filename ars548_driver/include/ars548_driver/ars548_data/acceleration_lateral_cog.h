#pragma once

#pragma pack(1)

#include "ars548_driver/util/byteswap.hpp"
#include <cstdint>

struct AccelerationLateralCoG
{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    float AccelerationLateralErrAmp;
    uint8_t AccelerationLateralErrAmp_InvalidFlag;
    uint8_t QualifierAccelerationLateral;
    float AccelerationLateral;
    uint8_t AccelerationLateral_InvalidFlag;
    uint8_t AccelerationLateralEventDataQualifier;
    uint64_t Reserved1;//
    uint64_t Reserved2;//For the message to be the right size
    uint32_t Reserved3;//

    inline void changeEndianness();
};

#pragma pack(4)

inline void AccelerationLateralCoG::changeEndianness() {
    ServiceID = byteswap(ServiceID);
    MethodID = byteswap(MethodID);
    PayloadLength = byteswap(PayloadLength);
    AccelerationLateralErrAmp = byteswap(AccelerationLateralErrAmp);
    AccelerationLateral = byteswap(AccelerationLateral);
    // Reserved1 = byteswap(Reserved1);
    // Reserved2 = byteswap(Reserved2);
    // Reserved3 = byteswap(Reserved3);
}
