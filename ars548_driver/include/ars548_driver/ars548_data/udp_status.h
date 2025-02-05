#include <ars548_driver/byteswap.hpp>
#include "ars548_messages/msg/status.hpp"

#define STATUS_MESSAGE_METHOD_ID 380
#define STATUS_MESSAGE_PAYLOAD 84
#define STATUS_MESSAGE_PDU_LENGTH 76

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

    //! @brief Checks for validity of the message (checks method ID and PayloadLength)
    inline bool isValid() const {
        return MethodID == STATUS_MESSAGE_METHOD_ID && PayloadLength == STATUS_MESSAGE_PAYLOAD;
    }

    inline void changeEndianness();
    inline ars548_messages::msg::Status toMsg();

    /**
     * 
     */
    inline bool receiveStatusMsg(int nbytes,const char * buffer){
        if(nbytes == STATUS_MESSAGE_PAYLOAD)
        {
            *this = *((struct UDPStatus *)buffer);
            changeEndianness();
            return isValid();
        }
        return false;
    }
};
#pragma pack(4)
/**
 * @brief Changes the Endiannes of the status struct.  (uint8_t fields don't need to)
 * 
 */
inline void UDPStatus::changeEndianness(){
    ServiceID = byteswap(ServiceID);
    MethodID = byteswap(MethodID);
    PayloadLength = byteswap(PayloadLength);
    Timestamp_Nanoseconds = byteswap(Timestamp_Nanoseconds);
    Timestamp_Seconds = byteswap(Timestamp_Seconds);
    Longitudinal = byteswap(Longitudinal);
    Lateral = byteswap(Lateral);
    Vertical = byteswap(Vertical);
    Yaw = byteswap(Yaw);
    Pitch = byteswap(Pitch);
    Length = byteswap(Length);
    Width = byteswap(Width);
    Height = byteswap(Height);
    Wheelbase = byteswap(Wheelbase);
    MaximunDistance = byteswap(MaximunDistance);
    SensorIPAddress_0 = byteswap(SensorIPAddress_0);
    SensorIPAddress_1 = byteswap(SensorIPAddress_1);
}

/**
     * @brief Fills the Status Messsage.
     * 
     * @param statusMessage The Status message to be filled.
     * @param status The Status struct used to fill the message.
     * 
     */
    inline ars548_messages::msg::Status UDPStatus::toMsg() {
        ars548_messages::msg::Status statusMessage;
        statusMessage.cycletime = CycleTime;
        statusMessage.configurationcounter = ConfigurationCounter;
        statusMessage.frequencyslot = FrequencySlot;
        statusMessage.hcc = HCC;
        statusMessage.height = Height;
        statusMessage.lateral = Lateral;
        statusMessage.length = Length;
        statusMessage.longitudinal = Longitudinal;
        statusMessage.maximundistance = MaximunDistance;
        statusMessage.pitch = Pitch;
        statusMessage.plugorientation = PlugOrientation;
        statusMessage.powersave_standstill = PayloadLength;
        statusMessage.sensoripaddress_0 = SensorIPAddress_0;
        statusMessage.sensoripaddress_1 = SensorIPAddress_1;
        statusMessage.status_blockagestatus = Status_BlockageStatus;
        statusMessage.status_characteristicspeed = Status_CharacteristicSpeed;
        statusMessage.status_drivingdirection = Status_DrivingDirection;
        statusMessage.status_lateralacceleration = Status_LateralAcceleration;
        statusMessage.status_longitudinalacceleration = Status_LongitudinalAcceleration;
        statusMessage.status_longitudinalvelocity = Status_LongitudinalVelocity;
        statusMessage.status_radarstatus = Status_RadarStatus;
        statusMessage.status_steeringangle = Status_SteeringAngle;
        statusMessage.status_temperaturestatus = Status_TemperatureStatus;
        statusMessage.status_voltagestatus = Status_VoltageStatus;
        statusMessage.status_yawrate = Status_YawRate;
        statusMessage.swversion_major = SWVersion_Major;
        statusMessage.swversion_minor = SWVersion_Minor;
        statusMessage.swversion_patch = SWVersion_Patch;
        statusMessage.timeslot = TimeSlot;
        statusMessage.timestamp_nanoseconds = Timestamp_Nanoseconds;
        statusMessage.timestamp_seconds = Timestamp_Seconds;
        statusMessage.timestamp_syncstatus = Timestamp_SyncStatus;
        statusMessage.vertical = Vertical;
        statusMessage.wheelbase = Wheelbase;
        statusMessage.width = Width;
        statusMessage.yaw = Yaw;

        return statusMessage;
    }
