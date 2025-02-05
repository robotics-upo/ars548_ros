#pragma once

#include "ars548_driver/util/byteswap.hpp"
#include <cstdint>

#pragma pack(1)

struct CharacteristicSpeed
{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    uint8_t CharacteristicSpeedErrAmp;
    uint8_t QualifierCharacteristicSpeed;
    uint8_t CharacteristicSpeed;
    uint64_t Reserved;//For the message to be the right size

    inline void changeEndianness();
};

#pragma pack(4)

void CharacteristicSpeed::changeEndianness() {
    ServiceID = byteswap(ServiceID);
    MethodID = byteswap(MethodID);
    PayloadLength = byteswap(PayloadLength);
    // Reserved = byteswap(Reserved);
}