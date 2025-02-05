#pragma once
#include <cstdint>
#include <ars548_driver/util/byteswap.hpp>

#pragma pack(1)

struct VelocityVehicle
{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    uint8_t StatusVelocityNearStandstill;
    uint8_t QualifierVelocityVehicle;
    uint8_t VelocityVehicleEventDataQualifier;
    float VelocityVehicle;
    uint8_t VelocityVehicle_InvalidFlag;
    uint64_t Reserved1;//
    uint64_t Reserved2;//For the message to be the right size
    uint32_t Reserved3;//

    inline void changeEndianness();
};

#pragma pack(4)

inline void VelocityVehicle::changeEndianness() {
    ServiceID = byteswap(ServiceID);
    MethodID = byteswap(MethodID);
    PayloadLength = byteswap(PayloadLength);

    VelocityVehicle = byteswap(VelocityVehicle);
    
}