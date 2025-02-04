#include <ars548_driver/byteswap.hpp>

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

    inline void changeEndianness();
};
#pragma pack(4)
/**
 * @brief Changes the Endiannes of the status struct.  (uint8_t fields don't need to)
 * 
 */
inline void UDPStatus::changeEndianness(){
    Timestamp_Nanoseconds=byteswap(Timestamp_Nanoseconds);
    Timestamp_Seconds=byteswap(Timestamp_Seconds);
    Longitudinal=byteswap(Longitudinal);
    Lateral=byteswap(Lateral);
    Vertical=byteswap(Vertical);
    Yaw=byteswap(Yaw);
    Pitch=byteswap(Pitch);
    Length=byteswap(Length);
    Width=byteswap(Width);
    Height=byteswap(Height);
    Wheelbase=byteswap(Wheelbase);
    MaximunDistance=byteswap(MaximunDistance);
    SensorIPAddress_0=byteswap(SensorIPAddress_0);
    SensorIPAddress_1=byteswap(SensorIPAddress_1);
}
