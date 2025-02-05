#pragma once

#include <cstdint>
#include <ars548_driver/util/byteswap.hpp>

#pragma pack(1)

struct DrivingDirection
{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    uint8_t DrivingDirectionUnconfirmed;
    uint8_t DrivingDirectionConfirmed;
    uint64_t Reserved1;//
    uint64_t Reserved2;//For the message to be the right size
    uint32_t Reserved3;//

    inline void changeEndianness();
};

#pragma pack(4)

inline void DrivingDirection::changeEndianness() {
    ServiceID = byteswap(ServiceID);
    MethodID = byteswap(MethodID);
    PayloadLength = byteswap(PayloadLength);
    // Reserved1 = byteswap(Reserved1);
    // Reserved2 = byteswap(Reserved2);
    // Reserved3 = byteswap(Reserved3);
}
