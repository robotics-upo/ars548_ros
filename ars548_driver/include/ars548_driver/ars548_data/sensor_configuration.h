#pragma once
#include "ars548_driver/byteswap.hpp"

#pragma pack(1)

struct SensorConfiguration
{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    float Longitudinal;
    float Lateral;
    float Vertical;
    float Yaw;
    float Pitch;
    uint8_t PlugOrientation;
    float Length;
    float Width;
    float Height;
    float Wheelbase;
    uint16_t MaximumDistance;
    uint8_t FrequencySlot;
    uint8_t CycleTime;
    uint8_t TimeSlot;
    uint8_t HCC;
    uint8_t Powersave_Standstill;
    uint32_t SensorIPAddress_0;
    uint32_t SensorIPAddress_1;
    uint8_t NewSensorMounting;
    uint8_t NewVehicleParameters;
    uint8_t NewRadarParameters;
    uint8_t NewNetworkConfiguration;

    inline void changeEndianness();
};
#pragma pack(4)

// @brief Changes the endianness of the data structure (uint8_t fields don't need to)
inline void SensorConfiguration::changeEndianness() {
    ServiceID = byteswap(ServiceID);
    MethodID = byteswap(MethodID);
    PayloadLength = byteswap(PayloadLength);
    Longitudinal = byteswap(Longitudinal);
    Lateral = byteswap(Lateral);
    Vertical = byteswap(Vertical);
    Yaw = byteswap(Yaw);
    Pitch = byteswap(Pitch);
// PlugOrientation = byteswap(PlugOrientation);
    Length = byteswap(Length);
    Width = byteswap(Width);
    Height = byteswap(Height);
    Wheelbase = byteswap(Wheelbase);
    MaximumDistance = byteswap(MaximumDistance);
    // FrequencySlot = byteswap(FrequencySlot);
    // CycleTime = byteswap(CycleTime);
    // TimeSlot = byteswap(TimeSlot);
    // HCC = byteswap(HCC);
    // Powersave_Standstill = byteswap(Powersave_Standstill);
    SensorIPAddress_0 = byteswap(SensorIPAddress_0);
    SensorIPAddress_1 = byteswap(SensorIPAddress_1);
    // NewSensorMounting = byteswap(NewSensorMounting);
    // NewVehicleParameters = byteswap(NewVehicleParameters);
    // NewRadarParameters = byteswap(NewRadarParameters);
    // NewNetworkConfiguration = byteswap(NewNetworkConfiguration);
}
