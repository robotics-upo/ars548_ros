/**
 * @file ars548_data.h
 * @brief ars548_data contains all of the Structures needed to save and modify the data from the radar.
 * It is also used to fill the messages sent to the user.
 * @brief Data obtained from the RadarSensors_Annex_AES548_IO SW 05.48.04.pdf
 */
#include <cstdint>
#pragma pack(1)
struct UDPStatus {
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    uint32_t Timestamp_Nanoseconds;
    uint32_t Timestamp_Seconds;
    uint8_t Timestamp_SyncStatus;
    uint8_t SWVersion_Major;
    uint8_t SWVersion_Minor;
    uint8_t SWVersion_Patch;
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
    uint16_t MaximunDistance;
    uint8_t FrequencySlot;
    uint8_t CycleTime;
    uint8_t TimeSlot;
    uint8_t HCC;
    uint8_t Powersave_Standstill;
    uint32_t SensorIPAddress_0;
    uint32_t SensorIPAddress_1;
    uint8_t ConfigurationCounter;
    uint8_t Status_LongitudinalVelocity;
    uint8_t Status_LongitudinalAcceleration;
    uint8_t Status_LateralAcceleration;
    uint8_t Status_YawRate;
    uint8_t Status_SteeringAngle;
    uint8_t Status_DrivingDirection;
    uint8_t Status_CharacteristicSpeed;
    uint8_t Status_RadarStatus;
    uint8_t Status_VoltageStatus;
    uint8_t Status_TemperatureStatus;
    uint8_t Status_BlockageStatus;
};

struct Object{
    uint16_t u_StatusSensor;
    uint32_t u_ID;
    uint16_t u_Age;
    uint8_t u_StatusMeasurement;
    uint8_t u_StatusMovement;
    uint16_t u_Position_InvalidFlags;
    uint8_t u_Position_Reference;
    float u_Position_X;
    float u_Position_X_STD;
    float u_Position_Y;
    float u_Position_Y_STD;
    float u_Position_Z;
    float u_Position_Z_STD;
    float u_Position_CovarianceXY;
    float u_Position_Orientation;
    float u_Position_Orientation_STD;
    uint8_t u_Existence_InvalidFlags;
    float u_Existence_Probability;
    float u_Existence_PPV;
    uint8_t u_Classification_Car;
    uint8_t u_Classification_Truck;
    uint8_t u_Classification_Motorcycle;
    uint8_t u_Classification_Bicycle;
    uint8_t u_Classification_Pedestrian;
    uint8_t u_Classification_Animal;
    uint8_t u_Classification_Hazard;
    uint8_t u_Classification_Unknown;
    uint8_t u_Classification_Overdrivable;
    uint8_t u_Classification_Underdrivable;
    uint8_t u_Dynamics_AbsVel_InvalidFlags;
    float f_Dynamics_AbsVel_X;
    float f_Dynamics_AbsVel_X_STD;
    float f_Dynamics_AbsVel_Y;
    float f_Dynamics_AbsVel_Y_STD;
    float f_Dynamics_AbsVel_CovarianceXY;
    uint8_t u_Dynamics_RelVel_InvalidFlags;
    float f_Dynamics_RelVel_X;
    float f_Dynamics_RelVel_X_STD;
    float f_Dynamics_RelVel_Y;
    float f_Dynamics_RelVel_Y_STD;
    float f_Dynamics_RelVel_CovarianceXY;
    uint8_t u_Dynamics_AbsAccel_InvalidFlags;
    float f_Dynamics_AbsAccel_X;
    float f_Dynamics_AbsAccel_X_STD;
    float f_Dynamics_AbsAccel_Y;
    float f_Dynamics_AbsAccel_Y_STD;
    float f_Dynamics_AbsAccel_CovarianceXY;
    uint8_t u_Dynamics_RelAccel_InvalidFlags;
    float f_Dynamics_RelAccel_X;
    float f_Dynamics_RelAccel_X_STD;
    float f_Dynamics_RelAccel_Y;
    float f_Dynamics_RelAccel_Y_STD;
    float f_Dynamics_RelAccel_CovarianceXY;
    uint8_t u_Dynamics_Orientation_InvalidFlags;
    float u_Dynamics_Orientation_Rate_Mean;
    float u_Dynamics_Orientation_Rate_STD;
    uint32_t u_Shape_Length_Status;
    uint8_t u_Shape_Length_Edge_InvalidFlags;
    float u_Shape_Length_Edge_Mean;
    float u_Shape_Length_Edge_STD;
    uint32_t u_Shape_Width_Status;
    uint8_t u_Shape_Width_Edge_InvalidFlags;
    float u_Shape_Width_Edge_Mean;
    float u_Shape_Width_Edge_STD;
};
struct Object_List{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    uint64_t empty1;//Because the data starts at bit 71
    uint64_t CRC;
    uint32_t Length;
    uint32_t SQC;
    uint32_t DataID;
    uint32_t Timestamp_Nanoseconds;
    uint32_t Timestamp_Seconds;
    uint8_t Timestamp_SyncStatus;
    uint32_t EventDataQualifier;
    uint8_t ExtendedQualifier;
    uint8_t ObjectList_NumOfObjects;
    struct Object ObjectList_Objects[50];
};
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
};

struct DetectionList{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    int64_t empty1;//Because the data starts at bit 71
    uint64_t CRC;
    uint32_t Length;
    uint32_t SQC;
    uint32_t DataID;
    uint32_t Timestamp_Nanoseconds;
    uint32_t Timestamp_Seconds;
    uint8_t Timestamp_SyncStatus;
    uint32_t EventDataQualifier;
    uint8_t ExtendedQualifier;
    uint16_t Origin_InvalidFlags;
    float Origin_Xpos;
    float Origin_Xstd;
    float Origin_Ypos;
    float Origin_Ystd;
    float Origin_Zpos;
    float Origin_Zstd;
    float Origin_Roll;
    float Origin_Rollstd;
    float Origin_Pitch;
    float Origin_Pitchstd;
    float Origin_Yaw;
    float Origin_Yawstd;
    uint8_t List_InvalidFlags;
    struct Detection List_Detections[800];
    float List_RadVelDomain_Min;
    float List_RadVelDomain_Max;
    uint32_t List_NumOfDetections;
    float Aln_AzimuthCorrection;
    float Aln_ElevationCorrection;
    uint8_t Aln_Status;
};
#pragma pack(4)
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


