/**
 * @file ars548_data.h
 * @brief ars548_data contains all of the Structures needed to save and modify the data from the radar.
 * It is also used to fill the messages sent to the user.
 * @brief Data obtained from the RadarSensors_Annex_AES548_IO SW 05.48.04.pdf
 */
#pragma once
#include <cstdint>


#pragma pack(1)


#define ARS548_MAX_DETECTIONS 2000
#include "ars548_data/udp_status.h"
#include "ars548_data/object_list.h"
#include "ars548_data/detection_list.h"


// CONFIGURATION MESSAGES
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
};
#pragma pack(4)
#pragma pack(1)
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
};
struct AccelerationLongitudinalCoG
{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    float AccelerationLongitudinalErrAmp;
    uint8_t AccelerationLongitudinalErrAmp_InvalidFlags;
    uint8_t QualifierAccelerationLongitudinal;
    float AccelerationLongitudinal;
    uint8_t AccelerationLongitudinal_InvalidFlag;
    uint8_t AccelerationLongitudinalEventDataQualifier;
    uint64_t Reserved1;//
    uint64_t Reserved2;//For the message to be the right size
    uint32_t Reserved3;//
};
struct CharacteristicSpeed
{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    uint8_t CharacteristicSpeedErrAmp;
    uint8_t QualifierCharacteristicSpeed;
    uint8_t CharacteristicSpeed;
    uint64_t Reserved;//For the message to be the right size
};
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
};
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
};
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
};
struct Yaw_Rate
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
};

#pragma pack(4)


