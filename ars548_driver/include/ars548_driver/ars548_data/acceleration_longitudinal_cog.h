#pragma once

#pragma pack(1)

#include "ars548_driver/util/byteswap.hpp"
#include <cstdint>

#pragma pack(1)

struct AccelerationLongitudinalCoG
{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    float AccelerationLongitudinalErrAmp;
    uint8_t AccelerationLongitudinalErrAmp_InvalidFlag;
    uint8_t QualifierAccelerationLongitudinal;
    float AccelerationLongitudinal;
    uint8_t AccelerationLongitudinal_InvalidFlag;
    uint8_t AccelerationLongitudinalEventDataQualifier;
    uint64_t Reserved1;//
    uint64_t Reserved2;//For the message to be the right size
    uint32_t Reserved3;//
    inline void changeEndianness();
};

#pragma pack(4)

inline void AccelerationLongitudinalCoG::changeEndianness() {
    ServiceID = byteswap(ServiceID);
    MethodID = byteswap(MethodID);
    PayloadLength = byteswap(PayloadLength);
    AccelerationLongitudinalErrAmp = byteswap(AccelerationLongitudinalErrAmp);
    AccelerationLongitudinalErrAmp_InvalidFlag = byteswap(AccelerationLongitudinalErrAmp_InvalidFlag);
    QualifierAccelerationLongitudinal = byteswap(QualifierAccelerationLongitudinal);
    AccelerationLongitudinal = byteswap(AccelerationLongitudinal);
    AccelerationLongitudinal_InvalidFlag = byteswap(AccelerationLongitudinal_InvalidFlag);
    AccelerationLongitudinalEventDataQualifier = byteswap(AccelerationLongitudinalEventDataQualifier);
    // Reserved1 = byteswap(Reserved1);
    // Reserved2 = byteswap(Reserved2);
    // Reserved3 = byteswap(Reserved3);

}