#pragma once
#include <cstdint>
#include <ars548_driver/util/byteswap.hpp>

#pragma pack(1)


struct YawRate
{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    float YawRateErrAmp;
    uint8_t YawRateErrAmp_InvalidFlag;
    uint8_t QualifierYawRate;
    float YawRate;
    uint8_t YawRate_InvalidFlag;
    uint8_t YawRateEventDataQualifier;
    uint64_t Reserved1;//
    uint64_t Reserved2;//For the message to be the right size
    uint32_t Reserved3;//

    inline void changeEndianness();
};

#pragma pack(4)

inline void YawRate::changeEndianness() {
    ServiceID = byteswap(ServiceID);
    MethodID = byteswap(MethodID);
    PayloadLength = byteswap(PayloadLength);

    YawRateErrAmp = byteswap(YawRateErrAmp);

    YawRate = byteswap(YawRate);
}