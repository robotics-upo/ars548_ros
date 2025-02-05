#pragma once
#include <cstdint>
#include <ars548_driver/util/byteswap.hpp>

#pragma pack(1)



struct SteeringAngleFrontAxle
{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    uint8_t QualifierSteeringAngleFrontAxle;
    float SteeringAngleFrontAxleErrAmp;
    uint8_t SteeringAngleFrontAxleErrAmp_InvalidFlag;
    float SteeringAngleFrontAxle;
    uint8_t SteeringAngleFrontAxle_InvalidFlag;
    uint8_t SteeringAngleFrontAxleEventDataQualifier;
    uint64_t Reserved1;//
    uint64_t Reserved2;//For the message to be the right size
    uint32_t Reserved3;//

    inline void changeEndianness();
};

#pragma pack(4)

inline void SteeringAngleFrontAxle::changeEndianness() {
    ServiceID = byteswap(ServiceID);
    MethodID = byteswap(MethodID);
    PayloadLength = byteswap(PayloadLength);

    SteeringAngleFrontAxleErrAmp = byteswap(SteeringAngleFrontAxleErrAmp);

    SteeringAngleFrontAxle = byteswap(SteeringAngleFrontAxle);
    // Reserved1 = byteswap(Reserved1);
    // Reserved2 = byteswap(Reserved2);
    // Reserved3 = byteswap(Reserved3);
}
