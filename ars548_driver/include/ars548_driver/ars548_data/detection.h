#pragma once
#include <cstdint>
#include <ars548_driver/byteswap.hpp>

#pragma pack(1)
struct Detection{
    float f_AzimuthAngle;
    float f_AzimuthAngleSTD;
    uint8_t u_InvalidFlags;
    float f_ElevationAngle;
    float f_ElevationAngleSTD;
    float f_Range;
    float f_RangeSTD;
    float f_RangeRate;
    float f_RangeRateSTD;
    int8_t s_RCS;
    uint16_t u_MeasurementID;
    uint8_t u_PositivePredictiveValue;
    uint8_t u_Classification;
    uint8_t u_MultiTargetProbabilityM;
    uint16_t u_ObjectID;
    uint8_t u_AmbiguityFlag;
    uint16_t u_SortIndex;

    inline void changeEndianness();
};

void Detection::changeEndianness() {
    f_AzimuthAngle = byteswap(f_AzimuthAngle);
    f_AzimuthAngleSTD = byteswap(f_AzimuthAngleSTD);
    f_ElevationAngle = byteswap(f_ElevationAngle);
    f_ElevationAngleSTD = byteswap(f_ElevationAngleSTD);
    f_Range = byteswap(f_Range);
    f_RangeSTD = byteswap(f_RangeSTD);
    f_RangeRate = byteswap(f_RangeRate);
    f_RangeRateSTD = byteswap(f_RangeRateSTD);
    u_MeasurementID = byteswap(u_MeasurementID);
    u_ObjectID = byteswap(u_ObjectID);
    u_SortIndex = byteswap(u_SortIndex);
}


#pragma pack(4)

