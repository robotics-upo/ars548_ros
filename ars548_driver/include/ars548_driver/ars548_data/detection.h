#pragma once
#include <cstdint>
#include <ars548_driver/util/byteswap.hpp>
#include <ars548_messages/msg/detection.hpp>

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

    inline ars548_messages::msg::Detection toMsg();
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

inline ars548_messages::msg::Detection Detection::toMsg() {
    ars548_messages::msg::Detection d;

    d.f_azimuthangle = f_AzimuthAngle;
    d.f_azimuthanglestd = f_AzimuthAngleSTD;
    d.f_elevationangle = f_ElevationAngle;
    d.f_elevationanglestd = f_ElevationAngleSTD;
    d.f_range = f_Range;
    d.f_rangerate = f_RangeRate;
    d.f_rangeratestd = f_RangeRateSTD;
    d.f_rangestd = f_RangeSTD;
    d.s_rcs = s_RCS;
    d.u_ambiguityflag = u_AmbiguityFlag;
    d.u_classification = u_Classification;
    d.u_invalidflags = u_InvalidFlags;
    d.u_measurementid = u_MeasurementID;
    d.u_multitargetprobabilitym = u_MultiTargetProbabilityM;
    d.u_objectid = u_ObjectID;
    d.u_positivepredictivevalue = u_PositivePredictiveValue;
    d.u_sortindex = u_SortIndex;   

    return d;
}
